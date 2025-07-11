# -*- coding: utf-8 -*-
"""
Main script for the multi-camera vision system for capturing
human demonstrations (pose, hands) relative to a detected object,
performing action segmentation, and visualizing in 2D/3D.
Includes Kalman Filter for object position smoothing and prediction.

Code developed by Sergio González Rodriguez
e-mail: meanssergy@gmail.com
Last modify: 10-04-2025
Licensed by: Santiago de Compostela University
"""
import os
import cv2
import numpy as np
import mediapipe as mp
import pyrealsense2 as rs
import open3d as o3d
# from collections import deque # Not used anymore
import csv # Used in vision_utils
from datetime import datetime
import time # For FPS and Kalman dt
import random # For colors

# <<< Import the auxiliary functions >>>
import vision_utils as vu

"""
--------------------------------------------------Configuration and prerequisites---------------------------------------
"""
# --- ONNX Model Configuration ---
# !!! Make sure this path is correct !!!
#ONNX_MODEL_PATH = '/home/sergio/Escritorio/PracticasTFG/LBDOARABOHG/vision/models/best.onnx'
ONNX_MODEL_PATH = '/home/sergio/Escritorio/PracticasTFG/Trayectoria/dataset_yolo_split/runs/detect/doll_yolov8n_run2/weights/best.onnx' # Using run 2
# !!! Make sure this file exists and has the correct content !!!
# (Eg: line 1: doll_head, line 2: doll_body)
CLASSES_PATH = './models/doll.names'
CONFIDENCE_THRESHOLD_YOLO = 0.4 # Threshold for YOLO detection
NMS_THRESHOLD_YOLO = 0.4      # Threshold for NMS in YOLO
INPUT_WIDTH_YOLO = 640        # Input width for YOLO (from export)
INPUT_HEIGHT_YOLO = 640       # Input height for YOLO (from export)
TARGET_CLASS_FOR_INTERACTION = "doll_head" # <<< CHANGED: Target class for interaction/distance/angle
SEPARATION_DISTANCE_THRESHOLD = 0.3 # <<< NEW: Threshold to consider head and body separated (tune this)

# Load YOLO Class Names
try:
    with open(CLASSES_PATH, 'r') as f:
        CLASS_NAMES_YOLO = [line.strip() for line in f.readlines()]
    # Generate random colors for each class for 2D drawing
    # random.seed(42) # Use fixed seed for consistent colors
    COLORS_YOLO = np.random.uniform(50, 200, size=(len(CLASS_NAMES_YOLO), 3))
    print(f"YOLO Classes loaded: {CLASS_NAMES_YOLO}")
except Exception as e:
    print(f"Error loading class names file '{CLASSES_PATH}': {e}")
    CLASS_NAMES_YOLO = []
    COLORS_YOLO = []

# Load ONNX Model with OpenCV DNN
net_yolo = None # Initialize as None
if CLASS_NAMES_YOLO: # Only try loading if classes were loaded
    try:
        net_yolo = cv2.dnn.readNetFromONNX(ONNX_MODEL_PATH)
        print(f"ONNX YOLO model loaded from: {ONNX_MODEL_PATH}")
        # Optional: Configure backend/target
        # net_yolo.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
        # net_yolo.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
    except cv2.error as e:
        print(f"Fatal Error loading ONNX model with OpenCV: {e}")
# --- End ONNX Model Configuration ---


# --- MediaPipe Configuration ---
mp_pose = mp.solutions.pose
pose_model = mp_pose.Pose(min_detection_confidence=0.7) 
mp_hands = mp.solutions.hands
hands_model = mp_hands.Hands(static_image_mode=False, max_num_hands=2, min_detection_confidence=0.4, min_tracking_confidence=0.4)
mp_drawing = mp.solutions.drawing_utils
# --- End MediaPipe Configuration ---


# --- RealSense Camera Configuration & Calibration ---
SERIAL_CAM1 = '153222070290'
SERIAL_CAM2 = '153222070548'
IMG_WIDTH = 640
IMG_HEIGHT = 480
FPS = 30
pipeline1 = rs.pipeline(); config1 = rs.config()
pipeline2 = rs.pipeline(); config2 = rs.config()
try:
    print(f"Configuring camera {SERIAL_CAM1}...")
    config1.enable_device(SERIAL_CAM1); config1.enable_stream(rs.stream.color, IMG_WIDTH, IMG_HEIGHT, rs.format.bgr8, FPS)
    print(f"Configuring camera {SERIAL_CAM2}...")
    config2.enable_device(SERIAL_CAM2); config2.enable_stream(rs.stream.color, IMG_WIDTH, IMG_HEIGHT, rs.format.bgr8, FPS)
    print("Starting RealSense pipelines...")
    pipeline1.start(config1); pipeline2.start(config2)
    print("Pipelines started.")
except Exception as e:
    print(f"Error initializing RealSense cameras: {e}"); exit()

# Load Calibration Data
calibration_dir = "./cal/data" # <<< Adjusted path - make sure this is correct relative to script location
try:
    cameraMatrix1 = np.load(os.path.join(calibration_dir, "camera_matrix_cam1.npy"))
    distCoeffs1 = np.load(os.path.join(calibration_dir, "dist_coeffs_cam1.npy"))
    cameraMatrix2 = np.load(os.path.join(calibration_dir, "camera_matrix_cam2.npy"))
    distCoeffs2 = np.load(os.path.join(calibration_dir, "dist_coeffs_cam2.npy"))
    R = np.load(os.path.join(calibration_dir, "R.npy"))
    T = np.load(os.path.join(calibration_dir, "T.npy"))
    P1 = cameraMatrix1 @ np.hstack((np.eye(3, 3), np.zeros((3, 1))))
    P2 = cameraMatrix2 @ np.hstack((R, T))
    print("Camera calibration files loaded successfully.")
except FileNotFoundError as e:
    print(f"Error loading calibration files from {calibration_dir}: {e}"); exit()
# --- End RealSense & Calibration ---


# --- Open3D Visualization Setup ---
vis = o3d.visualization.Visualizer()
vis.create_window(window_name="3D Visualization")
# Create geometries
pcd = o3d.geometry.PointCloud(); line_set = o3d.geometry.LineSet() # Pose
pcd_hands_left = o3d.geometry.PointCloud(); line_set4 = o3d.geometry.LineSet() # Left Hand
pcd_hands_right = o3d.geometry.PointCloud(); line_set3 = o3d.geometry.LineSet() # Right Hand
line_set2 = o3d.geometry.LineSet() # Object Box
# Initialize geometries with placeholder data
pcd.points = o3d.utility.Vector3dVector(np.zeros((len(vu.RELEVANT_POINTS), 3)))
pcd_hands_left.points = o3d.utility.Vector3dVector(np.zeros((21, 3)))
pcd_hands_right.points = o3d.utility.Vector3dVector(np.zeros((21, 3)))
line_set2.points = o3d.utility.Vector3dVector(np.zeros((4,3)))
line_set2.lines = o3d.utility.Vector2iVector(np.array([[0, 1], [1, 2], [2, 3], [3, 0]]))
# Add geometries to visualizer
vis.add_geometry(pcd); vis.add_geometry(line_set)
vis.add_geometry(pcd_hands_left); vis.add_geometry(line_set4)
vis.add_geometry(pcd_hands_right); vis.add_geometry(line_set3)
vis.add_geometry(line_set2)
mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
vis.add_geometry(mesh_frame)
view_ctl = None
try: # Poll events once to ensure window is responsive before getting control
    vis.poll_events(); vis.update_renderer(); view_ctl = vis.get_view_control()
    if view_ctl: print("View control obtained.")
except Exception as e: print(f"Exception getting view control: {e}")
# --- End Open3D Setup ---


# --- State Machine and Segmentation Variables ---
STATE_IDLE = "Idle"; STATE_GRASPING = "Grasping"; STATE_ROTATING = "Rotating";STATE_FINAL = "Final"
GRASP_DISTANCE_THRESHOLD = 0.17 # Threshold for hand-to-HEAD distance
ROTATION_THRESHOLD_DEGREES = 40 # Hand rotation threshold
HAND_TO_MONITOR = "Right"       # Hand used for state triggers
# SEPARATION_DISTANCE_THRESHOLD defined in ONNX config section
current_action_state = STATE_IDLE
active_trajectory_buffers = {
    "left_shoulder": [], "left_elbow": [], "left_hand": [],
    "right_shoulder": [], "right_elbow": [], "right_hand": [],
    "right_hip": [], # <<< AÑADIDO
    "right_hand_rotation_angle": [] 
}
angle_at_grasp = None
segment_counter = 0
output_folder = f"action_segments_DEMO{datetime.now().strftime('%Y%m%d_%H%M%S')}"
os.makedirs(output_folder, exist_ok=True)
print(f"Saving action segments in: {output_folder}")
# --- End State Machine Variables ---


# --- Processing Variables ---
last_3d_center = None # Last RAW triangulated center of TARGET_CLASS_FOR_INTERACTION (Head)
last_valid_box = None # Last RAW triangulated box corners of TARGET_CLASS_FOR_INTERACTION (Head)
# Optical Flow variables
lk_params = dict(winSize=(21, 21), maxLevel=0, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
prev_gray1, prev_gray2 = None, None
prev_points1l, prev_points1r, prev_points2l, prev_points2r = None, None, None, None
last_valid_hands_left_3d = np.zeros((21, 3)); last_valid_hands_right_3d = np.zeros((21, 3))
left_hand_source = "None"; right_hand_source = "None"
# --- End Processing Variables ---


# --- Kalman Filter Initialization for Target Object (Head) Tracking ---
kalman = cv2.KalmanFilter(6, 3) # State: [x,y,z, vx,vy,vz], Measurement: [x,y,z]
kalman.measurementMatrix = np.array([[1,0,0,0,0,0], [0,1,0,0,0,0], [0,0,1,0,0,0]], np.float32)
kalman.transitionMatrix = np.array([[1,0,0,1,0,0], [0,1,0,0,1,0], [0,0,1,0,0,1],
                                    [0,0,0,1,0,0], [0,0,0,0,1,0], [0,0,0,0,0,1]], np.float32)
kalman.processNoiseCov = np.eye(6, dtype=np.float32) * 1e-4 # Tune Q
kalman.measurementNoiseCov = np.eye(3, dtype=np.float32) * 1e-2 # Tune R
kalman.errorCovPost = np.eye(6, dtype=np.float32) * 0.1
kalman.statePost = np.zeros((6, 1), dtype=np.float32)
kalman_initialized = False
last_measurement_time = time.time()
# --- End Kalman Filter Initialization ---

# Variable to store previous unwrapped angle for continuity
prev_unwrapped_angle = None
angle_filter = vu.StreamingMovingAverage(window_size=3) # <<< AÑADIDO
print("\nStarting Main Loop...")
print("\n--- INSTRUCCIONES ---")
print(" 'd' -> Iniciar nueva demostración")
print(" 'f' -> Finalizar y guardar demostración actual")
print(" 'q' -> Salir del programa")
print("---------------------\n")
print("Esperando para iniciar la primera demostración... Presiona 'd'.")
# --- <<< NUEVO: Estado del Flujo de Grabación >>> ---
is_recording = False # Empieza en modo de espera
demo_counter = 0 # Contador para carpetas de demostraciones
#----------------------------------------------------- MAIN LOOP ---------------------------------------------------------------------------
frame_count = 0
while True:
    frame_count += 1
    current_time = time.time()
    dt = current_time - last_measurement_time # Time delta for Kalman filter
    last_measurement_time = current_time

    # --- 1. Get Frames ---
    try:
        frames1 = pipeline1.wait_for_frames(timeout_ms=1000); frames2 = pipeline2.wait_for_frames(timeout_ms=1000)
        if not frames1 or not frames2: print("Warning: Frames skip"); continue
        color_frame1 = frames1.get_color_frame(); color_frame2 = frames2.get_color_frame()
        if not color_frame1 or not color_frame2: print("Warning: Color frame skip"); continue
        image1 = np.asanyarray(color_frame1.get_data()); image2 = np.asanyarray(color_frame2.get_data())
        gray1 = cv2.cvtColor(image1, cv2.COLOR_BGR2GRAY); gray2 = cv2.cvtColor(image2, cv2.COLOR_BGR2GRAY)
    except Exception as e: print(f"Error getting frames: {e}"); time.sleep(0.1); continue

    # --- 2. 2D Detections (Object, Pose, Hands) ---
    if net_yolo is not None and CLASS_NAMES_YOLO:
        objects1 = vu.detect_objects(image1, net_yolo, CONFIDENCE_THRESHOLD_YOLO, NMS_THRESHOLD_YOLO, INPUT_WIDTH_YOLO, INPUT_HEIGHT_YOLO, CLASS_NAMES_YOLO)
        objects2 = vu.detect_objects(image2, net_yolo, CONFIDENCE_THRESHOLD_YOLO, NMS_THRESHOLD_YOLO, INPUT_WIDTH_YOLO, INPUT_HEIGHT_YOLO, CLASS_NAMES_YOLO)
        if frame_count % 60 == 0 and is_recording: print(f"Frame {frame_count}: YOLO Detections Cam1={len(objects1)}, Cam2={len(objects2)}")
    else: objects1, objects2 = [], []
    pts1, connections1 = vu.capture_pose_points(image1, pose_model)
    pts2, connections2 = vu.capture_pose_points(image2, pose_model)
    mp_hands1r_2d, mp_hands1l_2d, hlnd_list1 = vu.capture_hand_points(image1, hands_model)
    mp_hands2r_2d, mp_hands2l_2d, hlnd_list2 = vu.capture_hand_points(image2, hands_model)

    # --- 3. 2D Hand Tracking (Optical Flow Fallback) ---
    # ... (OF logic for left hand - same as before, updates hands1l_current, hands2l_current) ...
    hands1l_current, hands2l_current = None, None; left_hand_source = "None"
    if mp_hands1l_2d is not None and mp_hands2l_2d is not None: hands1l_current, hands2l_current, prev_points1l, prev_points2l, left_hand_source = mp_hands1l_2d, mp_hands2l_2d, mp_hands1l_2d, mp_hands2l_2d, "MediaPipe"
    elif prev_gray1 is not None and prev_gray2 is not None and prev_points1l is not None and prev_points2l is not None:
        try:
            next_points1l, status1, _ = cv2.calcOpticalFlowPyrLK(prev_gray1, gray1, prev_points1l, None, **lk_params); next_points2l, status2, _ = cv2.calcOpticalFlowPyrLK(prev_gray2, gray2, prev_points2l, None, **lk_params)
            if next_points1l is not None and status1 is not None and next_points2l is not None and status2 is not None:
                combined_status = (status1.ravel() == 1) & (status2.ravel() == 1); good_new1 = next_points1l[combined_status]; good_new2 = next_points2l[combined_status]
                if len(good_new1) > 10 and len(good_new2) > 10 : hands1l_current, hands2l_current, prev_points1l, prev_points2l, left_hand_source = good_new1.reshape(-1, 1, 2), good_new2.reshape(-1, 1, 2), good_new1.reshape(-1, 1, 2), good_new2.reshape(-1, 1, 2), "OpticalFlow"
                else: prev_points1l, prev_points2l, left_hand_source = None, None, "TrackingFailed"
            else: prev_points1l, prev_points2l, left_hand_source = None, None, "TrackingFailed"
        except cv2.error as e: print(f"Error OF Left: {e}"); prev_points1l, prev_points2l, left_hand_source = None, None, "TrackingFailed"
    # ... (OF logic for right hand - same as before, updates hands1r_current, hands2r_current) ...
    hands1r_current, hands2r_current = None, None; right_hand_source = "None"
    if mp_hands1r_2d is not None and mp_hands2r_2d is not None: hands1r_current, hands2r_current, prev_points1r, prev_points2r, right_hand_source = mp_hands1r_2d, mp_hands2r_2d, mp_hands1r_2d, mp_hands2r_2d, "MediaPipe"
    elif prev_gray1 is not None and prev_gray2 is not None and prev_points1r is not None and prev_points2r is not None:
        try:
            next_points1r, status1r, _ = cv2.calcOpticalFlowPyrLK(prev_gray1, gray1, prev_points1r, None, **lk_params); next_points2r, status2r, _ = cv2.calcOpticalFlowPyrLK(prev_gray2, gray2, prev_points2r, None, **lk_params)
            if next_points1r is not None and status1r is not None and next_points2r is not None and status2r is not None:
                combined_status_r = (status1r.ravel() == 1) & (status2r.ravel() == 1); good_new1r = next_points1r[combined_status_r]; good_new2r = next_points2r[combined_status_r]
                if len(good_new1r) > 10 and len(good_new2r) > 10: hands1r_current, hands2r_current, prev_points1r, prev_points2r, right_hand_source = good_new1r.reshape(-1, 1, 2), good_new2r.reshape(-1, 1, 2), good_new1r.reshape(-1, 1, 2), good_new2r.reshape(-1, 1, 2), "OpticalFlow"
                else: prev_points1r, prev_points2r, right_hand_source = None, None, "TrackingFailed"
            else: prev_points1r, prev_points2r, right_hand_source = None, None, "TrackingFailed"
        except cv2.error as e: print(f"Error OF Right: {e}"); prev_points1r, prev_points2r, right_hand_source = None, None, "TrackingFailed"

    # --- 4. 3D Triangulation (Pose, Hands) ---
    points_3d = None
    if pts1 and pts2: points_3d = vu.triangulate_points(pts1, pts2, P1, P2)
    # Hands
    hands_left_3d = last_valid_hands_left_3d # Start with fallback
    if hands1l_current is not None and hands2l_current is not None:
        temp_left_3d = vu.triangulate_points(hands1l_current.reshape(-1, 2), hands2l_current.reshape(-1, 2), P1, P2)
        if temp_left_3d is not None and temp_left_3d.shape[0] == 21: hands_left_3d = temp_left_3d; last_valid_hands_left_3d = temp_left_3d
    left_hand_source = left_hand_source if left_hand_source != "None" else "LastValid"
    hands_right_3d = last_valid_hands_right_3d # Start with fallback
    if hands1r_current is not None and hands2r_current is not None:
        temp_right_3d = vu.triangulate_points(hands1r_current.reshape(-1, 2), hands2r_current.reshape(-1, 2), P1, P2)
        if temp_right_3d is not None and temp_right_3d.shape[0] == 21: hands_right_3d = temp_right_3d; last_valid_hands_right_3d = temp_right_3d
    right_hand_source = right_hand_source if right_hand_source != "None" else "LastValid"

    # ***********************************************************************
    # ***** START: BLOCK FOR YOLO OBJECT PROCESSING & KALMAN FILTER *****
    # ***********************************************************************

    # Find best detections for HEAD and BODY in both views
    detected_head1 = None; highest_conf_h1 = -1
    detected_body1 = None; highest_conf_b1 = -1
    for obj in objects1:
        _, _, _, _, cls_id_obj, cls_name_obj, conf_obj = obj
        if cls_name_obj == "doll_head" and conf_obj > highest_conf_h1:
            detected_head1 = obj; highest_conf_h1 = conf_obj
        elif cls_name_obj == "doll_body" and conf_obj > highest_conf_b1:
            detected_body1 = obj; highest_conf_b1 = conf_obj

    detected_head2 = None; highest_conf_h2 = -1
    detected_body2 = None; highest_conf_b2 = -1
    for obj in objects2:
        _, _, _, _, cls_id_obj, cls_name_obj, conf_obj = obj
        if cls_name_obj == "doll_head" and conf_obj > highest_conf_h2:
            detected_head2 = obj; highest_conf_h2 = conf_obj
        elif cls_name_obj == "doll_body" and conf_obj > highest_conf_b2:
            detected_body2 = obj; highest_conf_b2 = conf_obj

    # -- Update Kalman Filter for the HEAD position --
    kalman.transitionMatrix[0, 3] = dt # Update F with dt
    kalman.transitionMatrix[1, 4] = dt
    kalman.transitionMatrix[2, 5] = dt
    predicted_state = kalman.predict()
    obj_3d_predicted_head = predicted_state[:3].reshape(3) # Predicted [x,y,z] for head

    measurement_head_successful = False
    current_head_3d_raw = None # Raw triangulation for head this frame
    current_box_3d_points = last_valid_box # Use fallback for box visualization initially

    if detected_head1 is not None and detected_head2 is not None:
        x1_h1, y1_h1, x2_h1, y2_h1, _, _, _ = detected_head1
        x1_h2, y1_h2, x2_h2, y2_h2, _, _, _ = detected_head2
        center1_h_2d = ((x1_h1 + x2_h1) / 2, (y1_h1 + y2_h1) / 2)
        center2_h_2d = ((x1_h2 + x2_h2) / 2, (y1_h2 + y2_h2) / 2)
        head_3d_candidate = vu.triangulate_points([center1_h_2d], [center2_h_2d], P1, P2)

        if head_3d_candidate is not None:
            current_head_3d_raw = head_3d_candidate[0]
            last_3d_center = current_head_3d_raw # Update generic fallback with head pos

            # Kalman Measurement Update
            measurement = np.array([[current_head_3d_raw[0]], [current_head_3d_raw[1]], [current_head_3d_raw[2]]], dtype=np.float32)
            if not kalman_initialized:
                kalman.statePost[:3] = measurement; kalman.statePost[3:] = 0
                kalman.errorCovPost = np.eye(6, dtype=np.float32) * 0.1
                kalman_initialized = True; print("--- Kalman Filter Initialized (Head) ---")
            else:
                kalman.correct(measurement) # Correct using head measurement
            measurement_head_successful = True

            # Update 3D box based on head detection
            box_2d_1 = [(x1_h1, y1_h1), (x2_h1, y1_h1), (x2_h1, y2_h1), (x1_h1, y2_h1)]
            box_2d_2 = [(x1_h2, y1_h2), (x2_h2, y1_h2), (x2_h2, y2_h2), (x1_h2, y2_h2)]
            box_3d_points_candidate = vu.triangulate_points(box_2d_1, box_2d_2, P1, P2)
            if box_3d_points_candidate is not None and len(box_3d_points_candidate) == 4:
                current_box_3d_points = box_3d_points_candidate
                last_valid_box = current_box_3d_points

    # Get filtered head position (either corrected or predicted)
    if kalman_initialized:
        obj_3d_filtered_head = kalman.statePost[:3].reshape(3)
    else:
        obj_3d_filtered_head = None # Not initialized yet

    # Triangulate body position (raw, no filter for now)
    obj_3d_body_raw = None
    if detected_body1 is not None and detected_body2 is not None:
        x1_b1, y1_b1, x2_b1, y2_b1, _, _, _ = detected_body1
        x1_b2, y1_b2, x2_b2, y2_b2, _, _, _ = detected_body2
        center1_b_2d = ((x1_b1 + x2_b1) / 2, (y1_b1 + y2_b1) / 2)
        center2_b_2d = ((x1_b2 + x2_b2) / 2, (y1_b2 + y2_b2) / 2)
        body_3d_candidate = vu.triangulate_points([center1_b_2d], [center2_b_2d], P1, P2)
        if body_3d_candidate is not None:
            obj_3d_body_raw = body_3d_candidate[0]
            # We could have a last_3d_body_center if needed as fallback for separation check

    # Update global variables
    obj_3d = obj_3d_filtered_head  # Main interaction object is the FILTERED HEAD position
    box_3d_points = current_box_3d_points # Visualization box is based on head detection / fallback
    # obj_3d_body is now available for the state machine (can be None)

    # *********************************************************************
    # ***** END: BLOCK FOR YOLO OBJECT PROCESSING & KALMAN FILTER *****
    # *********************************************************************


    # --- 5. Feature/State Calculation --- (Renumbered from original)
    # Get hand reference points
    left_hand_wrist_3d_hands = hands_left_3d[0] if hands_left_3d is not None and hands_left_3d.shape[0] > 0 and not np.all(hands_left_3d[0]==0) else None
    right_hand_wrist_3d_hands = hands_right_3d[0] if hands_right_3d is not None and hands_right_3d.shape[0] > 0 and not np.all(hands_right_3d[0]==0) else None
    right_wrist_3d_pose = points_3d[16] if points_3d is not None and len(points_3d) > 16 else None
    left_wrist_3d_pose = points_3d[15] if points_3d is not None and len(points_3d) > 15 else None

    left_hand_ref_3d = hands_left_3d[0] if hands_left_3d is not None and hands_left_3d.shape[0] > 0 and not np.all(hands_left_3d[0]==0) else None
    right_hand_ref_3d = hands_right_3d[0] if hands_right_3d is not None and hands_right_3d.shape[0] > 0 and not np.all(hands_right_3d[0]==0) else None
    monitored_hand_ref = right_hand_ref_3d if HAND_TO_MONITOR == "Right" else left_hand_ref_3d

    # Calculate unwrapped hand angle
    raw_angle_left = vu.estimate_hand_rotation(hands_left_3d)
    raw_angle_right = vu.estimate_hand_rotation(hands_right_3d)

    raw_monitored_angle = (
        raw_angle_right if HAND_TO_MONITOR == "Right" else raw_angle_left
    )

    current_unwrapped_angle = None

    if raw_monitored_angle is not None:
        if prev_unwrapped_angle is None:
            current_unwrapped_angle = raw_monitored_angle
        else:
            diff = raw_monitored_angle - prev_unwrapped_angle

            while diff <= -180:
                diff += 360

            while diff > 180:
                diff -= 360

            current_unwrapped_angle = prev_unwrapped_angle + diff
            prev_unwrapped_angle = current_unwrapped_angle
    else:
        prev_unwrapped_angle = None

    # Use unwrapped angle
    # Aplicar filtro si tenemos un valor válido
    filtered_angle = None
    if current_unwrapped_angle is not None:
        filtered_angle = angle_filter.process(current_unwrapped_angle)

    # Usa el ángulo filtrado para la máquina de estados y para guardar
    monitored_angle = filtered_angle 
    #monitored_angle = current_unwrapped_angle

    # Calculate Hand-to-HEAD distance (using FILTERED head position)
    distance = np.inf
    if monitored_hand_ref is not None and obj_3d is not None: # obj_3d is filtered head
        distance = np.linalg.norm(monitored_hand_ref - obj_3d)

        #print("Distancia entre mano y cabeza: ", distance)
    previous_state = current_action_state
    if is_recording:
        # --- 6. Accumulate Trajectories ---
        # Get points to save
        left_shoulder_3d = points_3d[11] if points_3d is not None and len(points_3d) > 11 else None
        left_elbow_3d = points_3d[13] if points_3d is not None and len(points_3d) > 13 else None
        right_shoulder_3d = points_3d[12] if points_3d is not None and len(points_3d) > 12 else None
        right_elbow_3d = points_3d[14] if points_3d is not None and len(points_3d) > 14 else None
        left_hand_to_save = hands_left_3d[0] if hands_left_3d is not None and not np.all(hands_left_3d[0]==0) else None
        right_hand_to_save = hands_right_3d[0] if hands_right_3d is not None and not np.all(hands_right_3d[0]==0) else None
        right_hip_3d = points_3d[24] if points_3d is not None and len(points_3d) > 24 else None # <<< AÑADIDO

        # Append to buffers, using None as a placeholder if data is missing for a frame
        active_trajectory_buffers["left_shoulder"].append(left_shoulder_3d.tolist() if left_shoulder_3d is not None else None)
        active_trajectory_buffers["left_elbow"].append(left_elbow_3d.tolist() if left_elbow_3d is not None else None)
        active_trajectory_buffers["left_hand"].append(left_hand_to_save.tolist() if left_hand_to_save is not None else None)
        active_trajectory_buffers["right_shoulder"].append(right_shoulder_3d.tolist() if right_shoulder_3d is not None else None)
        active_trajectory_buffers["right_elbow"].append(right_elbow_3d.tolist() if right_elbow_3d is not None else None)
        active_trajectory_buffers["right_hand"].append(right_hand_to_save.tolist() if right_hand_to_save is not None else None)
        active_trajectory_buffers["right_hip"].append(right_hip_3d.tolist() if right_hip_3d is not None else None) # <<< AÑADIDO

        if HAND_TO_MONITOR == "Right":
            active_trajectory_buffers["right_hand_rotation_angle"].append(monitored_angle)

        # --- 7. State Machine Logic & Segmented Saving ---
        previous_state = current_action_state
        if current_action_state == STATE_IDLE:
            if distance < GRASP_DISTANCE_THRESHOLD:
                print(f"Transition: {STATE_IDLE} -> {STATE_GRASPING} (Hand-Head Dist: {distance:.3f})")
                segment_counter += 1; vu.save_segmented_trajectories(f"Initial-{STATE_IDLE}", active_trajectory_buffers, output_folder, segment_counter)
                current_action_state = STATE_GRASPING; angle_at_grasp = monitored_angle
        elif current_action_state == STATE_GRASPING:
            if monitored_angle is not None and angle_at_grasp is not None and abs(monitored_angle - angle_at_grasp) > ROTATION_THRESHOLD_DEGREES:
                print(f"Transition: {STATE_GRASPING} -> {STATE_ROTATING} (Rotation detected: {abs(monitored_angle - angle_at_grasp):.1f}°)")
                segment_counter += 1; vu.save_segmented_trajectories(f"{STATE_GRASPING}-to-{STATE_ROTATING}", active_trajectory_buffers, output_folder, segment_counter); current_action_state = STATE_ROTATING
        elif current_action_state == STATE_ROTATING:
            distance_head_body = np.inf
            if obj_3d is not None and obj_3d_body_raw is not None: distance_head_body = np.linalg.norm(obj_3d - obj_3d_body_raw);
            if distance_head_body > SEPARATION_DISTANCE_THRESHOLD and distance_head_body != np.inf: 
                print(f"Transition: {STATE_ROTATING} -> {STATE_FINAL} (Head SEPARATED from body, Head-Body Dist: {distance_head_body:.3f})")
                segment_counter += 1; vu.save_segmented_trajectories(f"{STATE_ROTATING}-Separated", active_trajectory_buffers, output_folder, segment_counter); current_action_state = STATE_FINAL; angle_at_grasp = None
        elif current_action_state == STATE_FINAL:
            # En el estado final, esperamos la tecla 'f' para finalizar la demo. No hacemos nada aquí.
            pass
        if current_action_state != previous_state: print(f"--> Current State: {current_action_state}")

    elif current_action_state == STATE_FINAL:
        print("DEMOSTRACION FINALIZADA")
    if current_action_state != previous_state:
        print(f"--> Current State: {current_action_state}")

    # --- 8. Visualization (2D and 3D) ---
    processed_image1 = image1.copy(); processed_image2 = image2.copy()
    # --- 8a. 2D Drawing ---
    # Draw Pose
    if pts1: mp_drawing.draw_landmarks(processed_image1, pose_model.process(cv2.cvtColor(image1, cv2.COLOR_BGR2RGB)).pose_landmarks, mp_pose.POSE_CONNECTIONS)
    if pts2: mp_drawing.draw_landmarks(processed_image2, pose_model.process(cv2.cvtColor(image2, cv2.COLOR_BGR2RGB)).pose_landmarks, mp_pose.POSE_CONNECTIONS)
    # Draw Hands
    if hlnd_list1:
        for hand_landmarks in hlnd_list1: mp_drawing.draw_landmarks(processed_image1, hand_landmarks, mp_hands.HAND_CONNECTIONS)
    # ... (Opcional: Dibujar puntos OF para manos si no usa MP) ...
    if hlnd_list2:
        for hand_landmarks in hlnd_list2: mp_drawing.draw_landmarks(processed_image2, hand_landmarks, mp_hands.HAND_CONNECTIONS)
    # ... (Opcional: Dibujar puntos OF para manos si no usa MP) ...
    # Draw ALL Object Detections
    for obj in objects1:
        x1_obj, y1_obj, x2_obj, y2_obj, cls_id_obj, cls_name_obj, conf_obj = obj
        color_obj = COLORS_YOLO[cls_id_obj] if 0 <= cls_id_obj < len(COLORS_YOLO) else (200, 200, 200)
        cv2.rectangle(processed_image1, (x1_obj, y1_obj), (x2_obj, y2_obj), color_obj, 1)
        cv2.putText(processed_image1, f"{cls_name_obj}:{conf_obj:.2f}", (x1_obj, y1_obj - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color_obj, 1)
    for obj in objects2:
        x1_obj, y1_obj, x2_obj, y2_obj, cls_id_obj, cls_name_obj, conf_obj = obj
        color_obj = COLORS_YOLO[cls_id_obj] if 0 <= cls_id_obj < len(COLORS_YOLO) else (200, 200, 200)
        cv2.rectangle(processed_image2, (x1_obj, y1_obj), (x2_obj, y2_obj), color_obj, 1)
        cv2.putText(processed_image2, f"{cls_name_obj}:{conf_obj:.2f}", (x1_obj, y1_obj - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color_obj, 1)
    # Add Status Text
    cv2.putText(processed_image1, f"L:{left_hand_source}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
    cv2.putText(processed_image1, f"R:{right_hand_source}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
    cv2.putText(processed_image1, f"STATE: {current_action_state}", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
    # Show 2D Images
    cv2.imshow('Camera 1 - Pose & Objects', processed_image1)
    cv2.imshow('Camera 2 - Objects', processed_image2)


    # --- 8. Visualization (2D and 3D) ---
    # --- 8a. 2D Drawing ---
    processed_image1 = image1.copy() # Draw on copies
    processed_image2 = image2.copy()

    # Draw Pose
    if pts1: mp_drawing.draw_landmarks(processed_image1, pose_model.process(cv2.cvtColor(image1, cv2.COLOR_BGR2RGB)).pose_landmarks, mp_pose.POSE_CONNECTIONS)
    if pts2: mp_drawing.draw_landmarks(processed_image2, pose_model.process(cv2.cvtColor(image2, cv2.COLOR_BGR2RGB)).pose_landmarks, mp_pose.POSE_CONNECTIONS)

    # Draw Hands (from MediaPipe results or Optical Flow points)
    if hlnd_list1: # Use original MP landmarks for drawing if available
        for hand_landmarks in hlnd_list1: mp_drawing.draw_landmarks(processed_image1, hand_landmarks, mp_hands.HAND_CONNECTIONS)
    elif left_hand_source == "OpticalFlow" and hands1l_current is not None:
        for pt in hands1l_current.reshape(-1, 2): cv2.circle(processed_image1, tuple(map(int, pt)), 3, (0, 0, 255), -1)
    elif right_hand_source == "OpticalFlow" and hands1r_current is not None:
         for pt in hands1r_current.reshape(-1, 2): cv2.circle(processed_image1, tuple(map(int, pt)), 3, (255, 0, 0), -1)
    # Draw hands on image 2
    if hlnd_list2:
         for hand_landmarks in hlnd_list2: mp_drawing.draw_landmarks(processed_image2, hand_landmarks, mp_hands.HAND_CONNECTIONS)
    elif left_hand_source == "OpticalFlow" and hands2l_current is not None:
         for pt in hands2l_current.reshape(-1, 2): cv2.circle(processed_image2, tuple(map(int, pt)), 3, (0, 0, 255), -1)
    elif right_hand_source == "OpticalFlow" and hands2r_current is not None:
         for pt in hands2r_current.reshape(-1, 2): cv2.circle(processed_image2, tuple(map(int, pt)), 3, (255, 0, 0), -1)

    # Draw Object Detections (ALL detected objects)
    for obj in objects1:
        x1_obj, y1_obj, x2_obj, y2_obj, cls_id_obj, cls_name_obj, conf_obj = obj
        color_obj = COLORS_YOLO[cls_id_obj] if 0 <= cls_id_obj < len(COLORS_YOLO) else (200, 200, 200)
        cv2.rectangle(processed_image1, (x1_obj, y1_obj), (x2_obj, y2_obj), color_obj, 1)
        cv2.putText(processed_image1, f"{cls_name_obj}:{conf_obj:.2f}", (x1_obj, y1_obj - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color_obj, 1)
    for obj in objects2:
        x1_obj, y1_obj, x2_obj, y2_obj, cls_id_obj, cls_name_obj, conf_obj = obj
        color_obj = COLORS_YOLO[cls_id_obj] if 0 <= cls_id_obj < len(COLORS_YOLO) else (200, 200, 200)
        cv2.rectangle(processed_image2, (x1_obj, y1_obj), (x2_obj, y2_obj), color_obj, 1)
        cv2.putText(processed_image2, f"{cls_name_obj}:{conf_obj:.2f}", (x1_obj, y1_obj - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color_obj, 1)
    pose_angle_right_raw = vu.estimate_hand_rotation_from_pose(points_3d, hand="Right")
    # Add Status Text
    angle_hands_str = f"Angle (Hands): {monitored_angle:.1f}" if monitored_angle is not None else "Angle (Hands): N/A"
    cv2.putText(processed_image1, angle_hands_str, (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)

    angle_pose_str = f"Angle (Pose): {pose_angle_right_raw:.1f}" if pose_angle_right_raw is not None else "Angle (Pose): N/A"
    cv2.putText(processed_image1, angle_pose_str, (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 1)

    cv2.putText(processed_image1, f"L:{left_hand_source}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
    cv2.putText(processed_image1, f"R:{right_hand_source}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
    cv2.putText(processed_image1, f"STATE: {current_action_state}", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        # <<< MODIFICACIÓN: Añadir indicador de estado de grabación >>>
    if is_recording:
        rec_text = "[GRABANDO]"
        rec_color = (0, 0, 255) # Rojo
    else:
        rec_text = "[EN ESPERA]"
        rec_color = (0, 255, 0) # Verde
    cv2.putText(processed_image1, rec_text, (500, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, rec_color, 2)
    # --- FIN MODIFICACIÓN ---

    # Show 2D Images
    cv2.imshow('Camera 1 - Pose & Objects', processed_image1)
    cv2.imshow('Camera 2 - Objects', processed_image2)

    # --- 8b. 3D Visualization Update ---
    update_pose = False
    update_left_hand = False
    update_right_hand = False
    update_object = False

    # Update Pose Geometry
    if points_3d is not None and points_3d.shape[0] > 0:
        valid_indices = [i for i in vu.RELEVANT_POINTS if i < len(points_3d)]
        if valid_indices:
            filtered_points_3d = points_3d[valid_indices]
            if filtered_points_3d.shape[0] > 0:
                index_map = {original_idx: new_idx for new_idx, original_idx in enumerate(valid_indices)}
                filtered_connections = [(index_map[a], index_map[b]) for a, b in mp_pose.POSE_CONNECTIONS if a in index_map and b in index_map]
                try:
                    pcd.points = o3d.utility.Vector3dVector(np.asarray(filtered_points_3d, dtype=np.float64))
                    line_set.points = o3d.utility.Vector3dVector(np.asarray(filtered_points_3d, dtype=np.float64))
                    line_set.lines = o3d.utility.Vector2iVector(np.asarray(filtered_connections, dtype=np.int32) if filtered_connections else [])
                    update_pose = True
                except Exception as e: print(f"Error O3D Pose Update: {e}")

    # Update Object Box Geometry
    if box_3d_points is not None and box_3d_points.shape[0] == 4:
        try:
             line_set2.points = o3d.utility.Vector3dVector(np.asarray(box_3d_points, dtype=np.float64))
             lines = np.array([[0, 1], [1, 2], [2, 3], [3, 0], [0,2], [1,3]], dtype=np.int32) # Draw diagonals too
             line_set2.lines = o3d.utility.Vector2iVector(lines)
             update_object = True
        except Exception as e: print(f"Error O3D Object Update: {e}")

    # Update Left Hand Geometry
    if hands_left_3d is not None and hands_left_3d.shape[0] == 21:
         try:
             hand_connections = list(mp_hands.HAND_CONNECTIONS) if mp_hands.HAND_CONNECTIONS else []
             pcd_hands_left.points = o3d.utility.Vector3dVector(np.asarray(hands_left_3d, dtype=np.float64))
             line_set4.points = o3d.utility.Vector3dVector(np.asarray(hands_left_3d, dtype=np.float64))
             line_set4.lines = o3d.utility.Vector2iVector(np.asarray(hand_connections, dtype=np.int32) if hand_connections else [])
             color = [0, 1, 0];
             if left_hand_source == "OpticalFlow": color = [1, 1, 0]
             elif left_hand_source == "None" or left_hand_source == "TrackingFailed" or left_hand_source == "LastValid": color = [1, 0, 0]
             pcd_hands_left.paint_uniform_color(color)
             line_set4.paint_uniform_color(color)
             update_left_hand = True
         except Exception as e: print(f"Error O3D Left Hand Update: {e}")

    # Update Right Hand Geometry
    if hands_right_3d is not None and hands_right_3d.shape[0] == 21:
          try:
             hand_connections = list(mp_hands.HAND_CONNECTIONS) if mp_hands.HAND_CONNECTIONS else []
             pcd_hands_right.points = o3d.utility.Vector3dVector(np.asarray(hands_right_3d, dtype=np.float64))
             line_set3.points = o3d.utility.Vector3dVector(np.asarray(hands_right_3d, dtype=np.float64))
             line_set3.lines = o3d.utility.Vector2iVector(np.asarray(hand_connections, dtype=np.int32) if hand_connections else [])
             color = [0, 0.8, 0];
             if right_hand_source == "OpticalFlow": color = [1, 0.5, 0]
             elif right_hand_source == "None" or right_hand_source == "TrackingFailed" or right_hand_source == "LastValid": color = [0.8, 0, 0]
             pcd_hands_right.paint_uniform_color(color)
             line_set3.paint_uniform_color(color)
             update_right_hand = True
          except Exception as e: print(f"Error O3D Right Hand Update: {e}")

    # Perform Open3D Updates
    if update_pose:
        # Using add_geometry as workaround
        #vis.update_geometry(pcd)    
        #vis.update_geometry(line_set)
        vis.add_geometry(pcd) # Keep workaround if update fails visually
        vis.add_geometry(line_set)
    if update_object:
        vis.update_geometry(line_set2)
    if update_left_hand:
        vis.update_geometry(pcd_hands_left)
        vis.update_geometry(line_set4)
    if update_right_hand:
        vis.update_geometry(pcd_hands_right)
        vis.update_geometry(line_set3)

    # Update Renderer
    vis.poll_events()
    vis.update_renderer()
    # --- 9. End of Loop ---
 # --- 9. <<< MODIFICACIÓN: Control de Flujo de Grabación >>> ---
    key = cv2.waitKey(1) & 0xFF

    if key == ord('d'): # Iniciar/Reiniciar una nueva demostración
        demo_counter += 1
        print("\n" + "="*40)
        print(f"      INICIANDO NUEVA DEMOSTRACIÓN #{demo_counter}")
        print("="*40)
        # Resetear todo para un inicio limpio
        is_recording = True
        current_action_state = STATE_IDLE
        angle_at_grasp = None
        prev_unwrapped_angle = None
        segment_counter = 0
        # Crear una nueva carpeta para esta demostración específica
        output_folder = f"action_segments_DEMO-{demo_counter}_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        os.makedirs(output_folder, exist_ok=True)
        print(f"Guardando segmentos de la Demo #{demo_counter} en: {output_folder}")
        # Limpiar búferes
        for k in active_trajectory_buffers: active_trajectory_buffers[k].clear()
        
    elif key == ord('f'): # Finalizar la demostración actual
        if is_recording:
            print("\n" + "*"*40)
            print(f"    DEMOSTRACIÓN #{demo_counter} FINALIZADA Y GUARDADA")
            print("*"*40)
            is_recording = False
            # Guardar el último segmento activo al finalizar
            if any(active_trajectory_buffers.values()):
                 segment_counter += 1
                 final_phase_name = f"Final-{current_action_state}"
                 vu.save_segmented_trajectories(final_phase_name, active_trajectory_buffers, output_folder, segment_counter)
            # Volver al estado de espera
            current_action_state = STATE_IDLE
            print("\nSistema en [EN ESPERA]. Presiona 'd' para empezar la siguiente demo o 'q' para salir.")
        else:
            print("No hay ninguna grabación en curso para finalizar. Presiona 'd' para empezar una.")
    elif key == ord('c'):
        if is_recording:
            print("\n" + "!"*40)
            print(f"    DEMOSTRACIÓN #{demo_counter} CANCELADA. Datos descartados.")
            print("!"*40)
            is_recording = False
            # Simplemente limpiamos los búferes sin guardar
            for k in active_trajectory_buffers: active_trajectory_buffers[k].clear()
            current_action_state = STATE_IDLE
            print("\nSistema en [EN ESPERA]. Presiona 'd' para empezar de nuevo.")
        else:
            print("No hay ninguna grabación en curso para cancelar.")
    elif key == ord('q'): # Salir del programa
        if is_recording:
             print("Advertencia: Saliendo mientras una grabación estaba en curso. Guardando datos...")
             if any(active_trajectory_buffers.values()):
                 segment_counter += 1
                 final_phase_name = f"Final-{current_action_state}"
                 vu.save_segmented_trajectories(final_phase_name, active_trajectory_buffers, output_folder, segment_counter)
        print("Saliendo del programa.")
        break

# --- Cleanup ---
print("Stopping pipelines..."); pipeline1.stop(); pipeline2.stop()
print("Destroying windows..."); vis.destroy_window(); cv2.destroyAllWindows()
print("Process finished.")