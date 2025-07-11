# -*- coding: utf-8 -*-
"""
Utility functions for the Vision System.
Includes functions for camera capture, pose/hand/object detection,
3D triangulation, angle estimation, and segmented trajectory saving.
"""
import os
import cv2
import numpy as np
import mediapipe as mp
import csv
from datetime import datetime

# Define constants if they are only used by these functions
RELEVANT_POINTS = [11, 12, 13, 14, 15, 16, 23, 24] # Shoulder, elbow, wrist
import collections

class StreamingMovingAverage:
    """Calcula la media móvil de una secuencia de datos en tiempo real."""
    def __init__(self, window_size):
        if window_size <= 0: raise ValueError("El tamaño de la ventana debe ser un entero positivo.")
        self.window_size = window_size
        self.values = collections.deque(maxlen=window_size)
        self.sum = 0.0
    
    def process(self, value):
        if len(self.values) == self.window_size:
            self.sum -= self.values[0]
        self.values.append(value)
        self.sum += value
        return self.sum / len(self.values)
# --- MediaPipe Related Functions ---

def capture_pose_points(image, pose_model):
    """Processes an image with MediaPipe Pose to get 2D landmarks."""
    if image is None or pose_model is None:
        return [], []
    try:
        # Convert the BGR image to RGB
        rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        # Prevent modification of the original image
        rgb_image.flags.writeable = False
        # Process the image and find pose
        results = pose_model.process(rgb_image)
        # Restore writeable flag
        rgb_image.flags.writeable = True
        # Convert back to BGR (optional, if needed elsewhere)
        # image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)

        if results.pose_landmarks:
            h, w, _ = image.shape
            # Extract landmark coordinates (normalized -> pixels)
            points = [(lm.x * w, lm.y * h) for lm in results.pose_landmarks.landmark]
            # Get standard connections
            connections = list(mp.solutions.pose.POSE_CONNECTIONS)
            return points, connections
    except cv2.error as e:
        print(f"Error in cvtColor (Pose): {e}")
        return [], []
    except Exception as e:
        print(f"Error processing pose: {e}")
        return [], []
    return [], []

def capture_hand_points(image, hands_model):
    """Processes an image with MediaPipe Hands to get 2D landmarks for left/right."""
    left_hand_points = None
    right_hand_points = None
    mp_multi_hand_landmarks = None # To return original landmarks for drawing

    if image is None or hands_model is None:
        return left_hand_points, right_hand_points, mp_multi_hand_landmarks

    try:
        # Convert the BGR image to RGB
        rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        rgb_image.flags.writeable = False
        # Process the image and find hands
        results = hands_model.process(rgb_image)
        rgb_image.flags.writeable = True

        if results.multi_hand_landmarks and results.multi_handedness:
            mp_multi_hand_landmarks = results.multi_hand_landmarks # Save for potential drawing
            h, w, _ = image.shape
            for hand_landmarks, handedness in zip(results.multi_hand_landmarks, results.multi_handedness):
                hand_label = handedness.classification[0].label  # "Left" or "Right"
                # Extract landmarks and convert to numpy array (pixel coords)
                points = np.array([(lm.x * w, lm.y * h) for lm in hand_landmarks.landmark], dtype=np.float32)

                if hand_label == "Left":
                    # Format (N, 1, 2) for OpenCV Optical Flow
                    left_hand_points = points.reshape(-1, 1, 2)
                else: # Right
                    right_hand_points = points.reshape(-1, 1, 2)

    except cv2.error as e:
         print(f"Error in cvtColor (Hands): {e}")
    except Exception as e:
        print(f"Error processing hands: {e}")

    # Return points (can be None) and original landmarks
    return left_hand_points, right_hand_points, mp_multi_hand_landmarks

# --- Geometry and Math Functions ---

def compute_normal_vector(p0, p5, p17):
    """Computes the normalized normal vector given three 3D points."""
    # Check inputs are valid points (e.g., numpy arrays of size 3)
    if p0 is None or p5 is None or p17 is None: return None
    try:
        vector_a = p5 - p0
        vector_b = p17 - p0
        # Check for zero vectors
        norm_a = np.linalg.norm(vector_a)
        norm_b = np.linalg.norm(vector_b)
        if norm_a == 0 or norm_b == 0: return None # Cannot compute normal
        # Calculate cross product
        normal_vector = np.cross(vector_a, vector_b)
        norm_cross = np.linalg.norm(normal_vector)
        # Avoid division by zero if vectors are collinear
        if norm_cross < 1e-6: return None
        return normal_vector / norm_cross  # Return normalized normal
    except Exception as e:
        print(f"Error computing normal vector: {e}")
        return None


def estimate_hand_rotation(hand_3d):
    """Estimates hand rotation angle (XY projection) based on wrist, index, pinky."""
    # Check if hand_3d is a valid numpy array with enough points
    if hand_3d is not None and isinstance(hand_3d, np.ndarray) and hand_3d.shape[0] >= 18:
        # Use specific landmarks (Wrist=0, Index_MCP=5, Pinky_MCP=17)
        wrist = hand_3d[0]
        index_base = hand_3d[5]
        pinky_base = hand_3d[17]

        # Check if points are valid (e.g., not the initial zeros)
        if not np.all(wrist == 0) and not np.all(index_base == 0) and not np.all(pinky_base == 0):
            # Calculate normal vector using these points
            hand_normal = compute_normal_vector(wrist, index_base, pinky_base)
            if hand_normal is not None:
                # Calculate angle from XY projection of the normal
                angle_rad = np.arctan2(hand_normal[1], hand_normal[0])
                return angle_rad * 180 / np.pi # Return degrees
    return None # Return None if points are invalid, not enough points, or normal calculation failed

def triangulate_points(pts1, pts2, P1, P2):
    """Triangulates corresponding 2D points from two views into 3D."""
    # Input validation
    if pts1 is None or pts2 is None or len(pts1) == 0 or len(pts2) == 0 or len(pts1) != len(pts2):
        # print(f"Triangulation Error: Invalid input points.")
        return None

    # Ensure correct format (N, 2) and type for cv2.triangulatePoints
    try:
        pts1_arr = np.array(pts1, dtype=np.float32).reshape(-1, 2)
        pts2_arr = np.array(pts2, dtype=np.float32).reshape(-1, 2)
    except ValueError as e:
         print(f"Triangulation Error: Could not reshape input points. {e}")
         return None

    try:
        # Perform triangulation
        points_4d_hom = cv2.triangulatePoints(P1, P2, pts1_arr.T, pts2_arr.T)

        # Check if triangulation returned points
        if points_4d_hom is None or points_4d_hom.shape[1] == 0:
             # print("Warning: cv2.triangulatePoints returned no points.")
             return None

        # Convert from homogeneous to 3D coordinates
        w = points_4d_hom[3]
        # Avoid division by zero or very small 'w'
        valid_w_indices = np.where(np.abs(w) > 1e-6)[0]
        if len(valid_w_indices) == 0:
             # print("Warning: All triangulated points have near-zero 'w'.")
             return None # Or return zeros? For now, None.

        # Handle potentially invalid points (optional: could filter them out)
        # For simplicity, we might assume most points are valid if some are
        points_3d = np.zeros((3, points_4d_hom.shape[1]))
        valid_w = w[valid_w_indices]
        points_3d[:, valid_w_indices] = points_4d_hom[:3, valid_w_indices] / valid_w

        # Invert x and y coordinates for visualization convention
        points_3d[0, :] = -points_3d[0, :]  # Invert x
        points_3d[1, :] = -points_3d[1, :]  # Invert y

        return points_3d.T # Return as (N, 3)
    except cv2.error as e:
        print(f"Error in cv2.triangulatePoints: {e}")
        return None
    except Exception as e:
        print(f"Error during triangulation post-processing: {e}")
        return None

# --- Object Detection Function (ONNX version) ---

def detect_objects(image, net, conf_threshold, nms_threshold, input_width, input_height, class_names_map):
    """Detects objects using an ONNX model loaded with cv2.dnn."""
    if image is None or net is None: return []

    original_height, original_width = image.shape[:2]
    detections = [] # List to store final detections

    # 1. Preprocessing: Pad to square, resize, create blob
    try:
        length = max(original_height, original_width)
        img_padded = np.zeros((length, length, 3), np.uint8)
        img_padded[0:original_height, 0:original_width] = image
        # Calculate scale factor to map 640x640 coords back to padded image coords
        scale_factor = length / float(input_width)

        blob = cv2.dnn.blobFromImage(img_padded, scalefactor=1 / 255.0, size=(input_width, input_height), swapRB=True, crop=False)
    except Exception as e:
        print(f"Error during image preprocessing for YOLO: {e}")
        return []

    # 2. Inference
    try:
        net.setInput(blob)
        outputs = net.forward() # Raw output from the network
    except cv2.error as e:
        print(f"Error during ONNX inference (net.forward()): {e}")
        return []
    except Exception as e:
        print(f"Unexpected error during ONNX inference: {e}")
        return []

    # 3. Postprocessing (YOLOv8 specific output format assumed)
    try:
        # Output shape is typically (1, num_props, num_detections), e.g., (1, 6, 8400)
        # num_props = 4 (cx, cy, w, h) + num_classes
        if len(outputs.shape) != 3 or outputs.shape[0] != 1:
             print(f"Error: Unexpected output shape from network: {outputs.shape}")
             return []
        num_classes = len(class_names_map)
        expected_props = 4 + num_classes
        if outputs.shape[1] != expected_props:
             print(f"Error: Output properties ({outputs.shape[1]}) mismatch expected ({expected_props}) for {num_classes} classes.")
             return []

        outputs = outputs[0].T # Transpose to [num_detections, num_props] (e.g., [8400, 6])

        boxes = []        # Store candidate boxes [x, y, w, h] for NMS
        confidences = []  # Store candidate confidences for NMS
        class_ids = []    # Store candidate class IDs for NMS
        rows = outputs.shape[0]

        for i in range(rows):
            classes_scores = outputs[i][4:] # Scores start at index 4
            class_id = np.argmax(classes_scores)
            confidence = classes_scores[class_id]

            if confidence >= conf_threshold:
                # Box coords are cx, cy, w, h (normalized to input_width/height)
                cx, cy, w, h = outputs[i][:4]
                # Calculate top-left corner (x, y) relative to input size (e.g., 640x640)
                left = cx - (w / 2)
                top = cy - (h / 2)
                boxes.append([int(left), int(top), int(w), int(h)])
                confidences.append(float(confidence))
                class_ids.append(class_id)

        # 4. Apply Non-Maximum Suppression (NMS)
        indices = cv2.dnn.NMSBoxes(boxes, confidences, conf_threshold, nms_threshold) # Returns indices of boxes to keep

        # 5. Format final detections
        if len(indices) > 0:
            # Handle potential nested list structure from NMSBoxes
            if isinstance(indices, (list, tuple)) and len(indices) > 0 and isinstance(indices[0], (list, tuple)):
                 final_indices = np.array(indices).flatten()
            else:
                 final_indices = indices

            for i in final_indices:
                 # Ensure index is valid
                 if i < 0 or i >= len(boxes): continue

                 box = boxes[i]
                 conf = confidences[i]
                 class_id = class_ids[i]

                 # Descale box coordinates back to original image dimensions
                 # box = [left_640, top_640, w_640, h_640]
                 # scale_factor maps 640 -> padded image size (length)
                 x1 = int(box[0] * scale_factor)
                 y1 = int(box[1] * scale_factor)
                 w = int(box[2] * scale_factor)
                 h = int(box[3] * scale_factor)

                 # Clip coordinates to original image boundaries
                 x1 = max(0, x1)
                 y1 = max(0, y1)
                 x2 = min(original_width, x1 + w)
                 y2 = min(original_height, y1 + h)

                 # Get class name
                 class_name = class_names_map[class_id] if 0 <= class_id < len(class_names_map) else "unknown"

                 # Append detection if box is valid
                 if x2 > x1 and y2 > y1:
                     detections.append((x1, y1, x2, y2, class_id, class_name, conf))

    except Exception as e:
        print(f"Error during YOLO postprocessing: {e}")
        # import traceback # Optional: for detailed debugging
        # traceback.print_exc()

    return detections # Return list of (x1, y1, x2, y2, class_id, class_name, conf)

# --- Data Saving Function ---

#KEY_POINTS_TO_SAVE_FOR_ROBOT = ["right_shoulder", "right_elbow", "right_hand"] # Ajusta según necesites
# Asegúrate de que esta lista incluya todos los datos que quieres guardar
#KEY_POINTS_TO_SAVE_FOR_ROBOT = ["right_shoulder", "right_elbow", "right_hand", "right_hand_rotation_angle"]
# En vision_utils.py
KEY_POINTS_TO_SAVE_FOR_ROBOT = [
    "right_shoulder", "right_elbow", "right_hand", 
    "left_shoulder", "right_hip", # <<< AÑADIDO
    "right_hand_rotation_angle"
]
def save_segmented_trajectories(phase_name, active_trajectory_buffers, output_folder, segment_counter):
    """
    Saves all relevant keypoint trajectories for a given action phase into a SINGLE CSV file.
    Each row in the CSV represents a frame, and columns represent x,y,z for each keypoint.
    Handles scalar values (like angles) and None values gracefully.
    """
    # Lista de claves que realmente se guardarán (deben estar en la lista y en los buffers)
    keys_to_save = [kp for kp in KEY_POINTS_TO_SAVE_FOR_ROBOT if kp in active_trajectory_buffers]
    
    # Comprobar si hay datos para guardar en las claves seleccionadas
    if not any(active_trajectory_buffers.get(key) for key in keys_to_save):
        print(f"No data in buffers for relevant keys in phase {phase_name}, segment {segment_counter}. Skipping save.")
        return

    # Determinar el número de frames (todas las listas deben tener la misma longitud)
    # Esto está garantizado por la nueva lógica de acumulación que añade None
    num_frames = len(active_trajectory_buffers[keys_to_save[0]])
    
    timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]
    filename = f"segment_{segment_counter:03d}_{phase_name}_{timestamp_str}.csv"
    filepath = os.path.join(output_folder, filename)

    # Crear la cabecera del CSV
    header = []
    for key_point_name in keys_to_save:
        if "angle" in key_point_name.lower():
            header.append(f"{key_point_name}_value")
        else:
            header.extend([f"{key_point_name}_x", f"{key_point_name}_y", f"{key_point_name}_z"])
    
    print(f"Guardando segmento consolidado {segment_counter} ({phase_name}) con {num_frames} frames en: {filepath}")
    
    with open(filepath, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(header)

        for i in range(num_frames):
            row_data = []
            for key_point_name in keys_to_save:
                data_item = active_trajectory_buffers[key_point_name][i]

                if data_item is None:
                    # Escribir strings vacíos si el dato es None
                    if "angle" in key_point_name.lower():
                        row_data.append('') 
                    else:
                        row_data.extend(['', '', ''])
                elif "angle" in key_point_name.lower():
                    row_data.append(data_item)
                elif isinstance(data_item, list) and len(data_item) == 3:
                    row_data.extend(data_item)
                else:
                    # Dato inesperado, escribir Nones
                    if "angle" in key_point_name.lower(): row_data.append('')
                    else: row_data.extend(['', '', ''])
            
            writer.writerow(row_data)
    
    # La limpieza de búferes ahora se gestiona en el script principal al iniciar una nueva demo
    # Pero por si acaso, podemos limpiarlos aquí también después de guardar.
    for key in active_trajectory_buffers:
        active_trajectory_buffers[key].clear()
"""
def save_segmented_trajectories(phase_name, active_trajectory_buffers, output_folder, segment_counter):
    if not any(active_trajectory_buffers.values()):
        print(f"No data in buffers for phase {phase_name}, segment {segment_counter}. Skipping save.")
        return

    valid_lengths = []
    # Ensure all keys we want to save are actually in the buffers and have data
    # This also implicitly defines the order in the CSV
    keys_present_in_buffers = [kp for kp in KEY_POINTS_TO_SAVE_FOR_ROBOT if kp in active_trajectory_buffers and active_trajectory_buffers[kp]]

    if not keys_present_in_buffers:
        print(f"None of the specified KEY_POINTS_TO_SAVE_FOR_ROBOT are in buffers for phase {phase_name}. Skipping.")
        return

    for kp in keys_present_in_buffers:
         valid_lengths.append(len(active_trajectory_buffers[kp]))

    if not valid_lengths: # Should be redundant due to above check, but for safety
        print(f"No data for specified KEY_POINTS_TO_SAVE_FOR_ROBOT in phase {phase_name}. Skipping save.")
        return

    num_frames = min(valid_lengths)
    if num_frames == 0:
        print(f"Zero frames recorded for phase {phase_name}. Skipping save.")
        return

    timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]
    filename = f"segment_{segment_counter:03d}_{phase_name}_{timestamp_str}.csv"
    filepath = os.path.join(output_folder, filename)

    header = []
    for key_point_name in keys_present_in_buffers: # Iterate over keys actually present
        if "angle" in key_point_name.lower(): # Simple check for angle data
            header.append(f"{key_point_name}_value") # e.g., right_hand_rotation_angle_value
        else:
            header.extend([f"{key_point_name}_x", f"{key_point_name}_y", f"{key_point_name}_z"])

    print(f"Saving consolidated segment {segment_counter} ({phase_name}) with {num_frames} frames to {filepath}")
    #print(f"Header: {header}") # Debug print header

    with open(filepath, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(header)

        for i in range(num_frames):
            row_data = []
            for key_point_name in keys_present_in_buffers: # Iterate in the same order as header
                # It's crucial that active_trajectory_buffers[key_point_name] has at least 'i' elements
                # num_frames = min(lengths) should ensure this.
                data_item = active_trajectory_buffers[key_point_name][i]

                if data_item is None: # Handle None values by writing empty string
                    if "angle" in key_point_name.lower():
                        row_data.append('') 
                    else:
                        row_data.extend(['', '', ''])
                elif "angle" in key_point_name.lower(): # Scalar angle data
                    row_data.append(data_item) # Assumes data_item is a scalar
                elif isinstance(data_item, list) and len(data_item) == 3: # x,y,z point data
                    row_data.extend(data_item)
                else: # Unexpected data format for a point
                    print(f"Warning: Unexpected data format for point {key_point_name} at frame {i}: {data_item}. Writing Nones.")
                    row_data.extend(['', '', ''])

            writer.writerow(row_data)

    # Clear all buffers that were part of KEY_POINTS_TO_SAVE_FOR_ROBOT and were present
    for key_point_name in keys_present_in_buffers:
        active_trajectory_buffers[key_point_name].clear()
    # Also clear other buffers that might exist but weren't saved, to be clean for next segment
    other_keys = [k for k in active_trajectory_buffers if k not in keys_present_in_buffers]
    for k_other in other_keys:
        active_trajectory_buffers[k_other].clear()
"""
"""
def save_segmented_trajectories(phase_name, active_trajectory_buffers, output_folder, segment_counter):
    #Saves all relevant keypoint trajectories for a given action phase into a SINGLE CSV file.
    #Each row in the CSV represents a frame, and columns represent x,y,z for each keypoint.
    
    if not any(active_trajectory_buffers.values()): # Check if any buffer has data
        print(f"No data in buffers for phase {phase_name}, segment {segment_counter}. Skipping save.")
        return

    # Determine the number of frames (length of the shortest trajectory for safety, though they should be same)
    # Only consider trajectories for KEY_POINTS_TO_SAVE_FOR_ROBOT
    valid_lengths = [len(active_trajectory_buffers[kp]) for kp in KEY_POINTS_TO_SAVE_FOR_ROBOT if kp in active_trajectory_buffers and active_trajectory_buffers[kp]]
    if not valid_lengths:
        print(f"No data for specified KEY_POINTS_TO_SAVE_FOR_ROBOT in phase {phase_name}. Skipping save.")
        return
    num_frames = min(valid_lengths)
    if num_frames == 0:
        print(f"Zero frames recorded for phase {phase_name}. Skipping save.")
        return

    timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3] # Milliseconds
    filename = f"segment_{segment_counter:03d}_{phase_name}_{timestamp_str}.csv"
    filepath = os.path.join(output_folder, filename)

    # Create header
    header = []
    for key_point_name in KEY_POINTS_TO_SAVE_FOR_ROBOT:
        header.extend([f"{key_point_name}_x", f"{key_point_name}_y", f"{key_point_name}_z"])

    print(f"Saving consolidated segment {segment_counter} ({phase_name}) with {num_frames} frames to {filepath}")

    with open(filepath, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(header) # Write header

        for i in range(num_frames):
            row_data = []
            all_points_available_for_frame = True
            for key_point_name in KEY_POINTS_TO_SAVE_FOR_ROBOT:
                if key_point_name in active_trajectory_buffers and i < len(active_trajectory_buffers[key_point_name]):
                    point_xyz = active_trajectory_buffers[key_point_name][i] # point_xyz is [x, y, z]
                    if point_xyz is not None and len(point_xyz) == 3:
                         row_data.extend(point_xyz)
                    else: # Handle if a specific point is None for a frame (should not happen if logic is correct)
                        row_data.extend([None, None, None]) # Or some placeholder like np.nan
                        print(f"Warning: Missing data for {key_point_name} at frame {i} in segment {segment_counter}")
                        all_points_available_for_frame = False # Mark if any point is missing
                else: # Should not happen if num_frames is calculated from valid_lengths
                    row_data.extend([None, None, None])
                    print(f"Error: Data inconsistency for {key_point_name} at frame {i} in segment {segment_counter}")
                    all_points_available_for_frame = False


            if all_points_available_for_frame: # Only write row if all key points for this frame were valid
                writer.writerow(row_data)
            # else:
                # print(f"Skipping frame {i} in segment {segment_counter} due to missing point data.")


    # Clear buffers after saving (original behavior)
    for key_point_name in active_trajectory_buffers:
        active_trajectory_buffers[key_point_name].clear()
"""
# En vision_utils.py
def get_hand_orientation_matrix(hand_landmarks_3d):
    if hand_landmarks_3d is None or len(hand_landmarks_3d) < 21:
        return None

    p0 = hand_landmarks_3d[0]  # Wrist
    p5 = hand_landmarks_3d[5]  # INDEX_FINGER_MCP
    p17 = hand_landmarks_3d[17] # PINKY_MCP

    # Eje X (aproximadamente a lo largo de la mano, de muñeca a dedos)
    # Podrías usar un punto medio entre p5 y p9 (MIDDLE_FINGER_MCP) o p5 y p17
    # O más simple: p5 - p0
    x_axis = (p5 - p0) + (p17 - p0) # Suma de vectores para una dirección media
    if np.linalg.norm(x_axis) < 1e-6: return None
    x_axis = x_axis / np.linalg.norm(x_axis)

    # Eje Z (normal a la palma, usando el método de tu estimate_hand_rotation)
    v1 = p5 - p0  # Vector de muñeca a MCP del índice
    v2 = p17 - p0 # Vector de muñeca a MCP del meñique
    z_axis = np.cross(v1, v2)
    if np.linalg.norm(z_axis) < 1e-6: return None
    z_axis = z_axis / np.linalg.norm(z_axis)

    # Eje Y (producto cruzado para asegurar ortogonalidad)
    y_axis = np.cross(z_axis, x_axis)
    if np.linalg.norm(y_axis) < 1e-6: return None
    y_axis = y_axis / np.linalg.norm(y_axis)

    # Re-calcular eje X para que sea perfectamente ortogonal (opcional pero bueno)
    x_axis = np.cross(y_axis, z_axis)

    # Construir la matriz de rotación
    # Columnas son los ejes del nuevo sistema en términos del sistema original
    rotation_matrix = np.column_stack((x_axis, y_axis, z_axis))
    return rotation_matrix

# <<< NUEVA FUNCIÓN >>>
def estimate_hand_rotation_from_pose(pose_landmarks_3d, hand="Right"):
    """
    Calcula el ángulo de rotación de la mano usando los landmarks de MediaPipe Pose.
    """
    if pose_landmarks_3d is None or len(pose_landmarks_3d) < 23:
        return None

    try:
        # Índices para la mano derecha en MediaPipe Pose
        if hand == "Right":
            p_wrist = pose_landmarks_3d[16]
            p_index = pose_landmarks_3d[20]
            p_pinky = pose_landmarks_3d[18]
        # Índices para la mano izquierda
        elif hand == "Left":
            p_wrist = pose_landmarks_3d[15]
            p_index = pose_landmarks_3d[19]
            p_pinky = pose_landmarks_3d[17]
        else:
            return None

        # Asegurarse de que los puntos son válidos (no None y no [0,0,0])
        if any(p is None for p in [p_wrist, p_index, p_pinky]) or \
           any(np.all(p == 0) for p in [p_wrist, p_index, p_pinky]):
            return None

        # Vectores desde la muñeca a los nudillos del índice y meñique
        v1 = np.array(p_index) - np.array(p_wrist)
        v2 = np.array(p_pinky) - np.array(p_wrist)

        # Vector normal al plano de la palma (producto cruzado)
        normal = np.cross(v1, v2)
        normal = normal / np.linalg.norm(normal)

        # Proyección del vector normal en el plano XY para calcular el ángulo
        # (Esta lógica es la misma que en tu función original)
        angle = np.arctan2(normal[1], normal[0])
        return np.degrees(angle)

    except (IndexError, TypeError, ValueError) as e:
        # print(f"Error al calcular ángulo desde pose: {e}") # Para depurar
        return None