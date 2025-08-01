#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Main script for Learning from Demonstration (LfD) using Dynamic Movement Primitives (DMPs).

This script performs the complete offline learning pipeline:
1. Loads all captured human demonstration files (.csv).
2. Applies a robust kinematic mapping to translate human poses (camera space) into
   robot joint angle trajectories (robot joint space). This includes:
    - Applying a precise camera-to-robot calibration.
    - Solving the morphological differences using a "reflection" technique.
    - Calculating unambiguous human joint angles using a dynamic torso reference frame.
    - Mapping relative human movements to the robot's joint conventions.
3. Filters and resamples all translated trajectories.
4. Trains a DMP model for each distinct phase of the demonstrated task on the
   averaged trajectory of all demonstrations.
5. Saves the learned DMP models as .pkl files, ready for execution by the robot controller.

Developed by: Sergio González Rodriguez (meanssergy@gmail.com)
For: University of Santiago de Compostela (USC) - Final Degree Project
Date: August 2025
"""

import numpy as np
import pandas as pd
import os
import glob
import math
import pickle
from scipy.signal import savgol_filter
import pydmps.dmp_discrete
import sys

# --- 1. Configuration ---

# File paths for input data and output models.
DEMONSTRATIONS_BASE_DIR = "TFG_Sergio/vision_system/action_segments_ALL3/action_segments_DEMO-*"
CALIBRATION_MATRIX_FILE = "TFG_Sergio/auxiliary_tools/cam_cal/data/T_base_a_camara.npy"
PHASES_TO_LEARN = ["Initial-Idle", "Grasping-to-Rotating", "Rotating-Separated", "Final-Final"]
DMP_OUTPUT_DIR = "./learned_dmps_final2"  # Folder to save the learned DMP models.
os.makedirs(DMP_OUTPUT_DIR, exist_ok=True)

# Preprocessing and DMP model parameters.
NUM_RESAMPLE_POINTS = 200  # Resample all trajectories to this fixed length for averaging and learning.
N_BASIS_FUNCTIONS = 30     # Number of basis functions for the DMP model, defining its complexity.

# --- 2. Kinematic Mapping Configuration ---

# The human pose landmarks required from the CSV files for the full kinematic mapping.
KEY_POINTS_TO_PROCESS = ["right_shoulder", "right_elbow", "right_hand", "left_shoulder", "right_hip"]

# Reference poses for the human-robot mapping (based on the technical report, Sec. 3.2).
# These define the starting offset for mapping relative movements.
ROBOT_CALIB_ANGLES = {'shoulder_pan_joint': 0.0, 'shoulder_lift_joint': -math.pi/2, 'elbow_joint': 0.0}
HUMAN_CALIB_ANGLES_TPose = {'shoulder_flex': 0.0, 'shoulder_abd': 90.0, 'elbow': 180.0}
CALIBRATION_DATA = {'robot': ROBOT_CALIB_ANGLES, 'human': HUMAN_CALIB_ANGLES_TPose}

# Scale factors to invert joint directions, correcting for different kinematic conventions.
SCALE_FACTORS = {'shoulder_pan': -1.0, 'shoulder_lift': -1.0, 'elbow': -1.0, 'wrist_1': -1.0}

# Default values for robot wrist joints.
ROBOT_WRIST_1_AT_GRASP_REF_RAD, ROBOT_WRIST_2_FIXED_RAD, ROBOT_WRIST_3_FIXED_RAD = 0.0, 0.0, 0.0
HUMAN_ANGLE_AT_GRASP_REF_DEG = None # Global variable to store the hand angle at the start of rotation.


# --- 3. Helper Functions ---

def find_all_demonstration_folders(base_dir_pattern):
    """
    Finds and numerically sorts all demonstration folders matching a pattern.
    
    This ensures that demonstrations are processed in the correct order (e.g., DEMO-10
    comes after DEMO-9, not after DEMO-1).

    Args:
        base_dir_pattern (str): A glob path pattern to search for folders.
                                (e.g., "data/DEMO-*")
    
    Returns:
        list: A numerically sorted list of folder paths.
    """
    list_of_folders = glob.glob(base_dir_pattern)
    if not list_of_folders:
        return []
    
    # Inner helper function to extract the integer number from the folder name.
    def get_demo_number(path):
        try:
            return int(path.split('DEMO-')[1].split('_')[0])
        except (IndexError, ValueError):
            return -1 # Return -1 for improperly named folders.
            
    return sorted(list_of_folders, key=get_demo_number)


def load_and_filter_phase_from_demo(demo_folder_path, phase_name_pattern):
    """
    Loads, cleans, and filters the CSV file for a specific action phase from a
    single demonstration folder.

    This function is critical for data quality. It handles missing values (NaNs) by
    interpolating and smooths the trajectory using a Savitzky-Golay filter to
    remove noise from the vision system.

    Args:
        demo_folder_path (str): The path to a single demonstration folder.
        phase_name_pattern (str): The name pattern of the phase to load (e.g., "Grasping-*").

    Returns:
        pd.DataFrame: A cleaned and filtered pandas DataFrame, or None if an error occurs.
    """
    search_pattern = os.path.join(demo_folder_path, f"segment_*_{phase_name_pattern}_*.csv")
    matching_files = sorted(glob.glob(search_pattern))
    if not matching_files:
        return None
        
    try:
        df = pd.read_csv(matching_files[0])
        
        # --- Data Cleaning ---
        # Interpolate to fill any NaN gaps in the data.
        for col in df.columns:
            if df[col].isnull().any():
                df[col] = df[col].interpolate(method='linear', limit_direction='both')
        df = df.dropna() # Drop any rows that could not be filled.
        
        # The filter requires a minimum number of data points.
        if len(df) <= 11:
            return None

        # --- Noise Filtering ---
        # Apply a Savitzky-Golay filter to all numeric columns to smooth the trajectory.
        for col in df.columns:
            if pd.api.types.is_numeric_dtype(df[col]):
                df[col] = savgol_filter(df[col], 11, 3)
        return df
    except Exception as e:
        print(f"  -> Error while filtering or processing {matching_files[0]}: {e}")
        return None


def get_angle_between_vectors(v1, v2):
    """
    Calculates the angle in degrees between two 3D vectors.

    Args:
        v1 (np.array): The first vector.
        v2 (np.array): The second vector.

    Returns:
        float: The angle in degrees.
    """
    # Normalize vectors to ensure the dot product gives the cosine of the angle.
    norm_v1, norm_v2 = np.linalg.norm(v1), np.linalg.norm(v2)
    if norm_v1 < 1e-6 or norm_v2 < 1e-6:
        return 0.0 # Avoid division by zero.
    v1_u, v2_u = v1 / norm_v1, v2 / norm_v2
    
    # Calculate angle using the dot product formula. Clip ensures the value is in [-1, 1].
    return np.degrees(np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0)))


def map_human_to_robot_angles(human_angles, human_wrist_angle):
    """
    Maps the calculated human joint angles to the target robot joint angles.
    
    This function implements the relative mapping methodology from the technical report (Sec 3.2).
    It calculates the deviation from a reference T-Pose and applies that deviation to the
    robot's reference "Home" pose.

    Args:
        human_angles (dict): A dictionary with the current calculated human angles
                             {'elbow', 'shoulder_flex', 'shoulder_abd'}.
        human_wrist_angle (float): The current rotation angle of the human wrist in degrees.

    Returns:
        list: A list of 6 target joint angles for the UR3e robot in radians.
    """
    global HUMAN_ANGLE_AT_GRASP_REF_DEG
    h_calib = CALIBRATION_DATA['human']
    r_calib = CALIBRATION_DATA['robot']
    
    # Calculate the angular deviation of the human from their reference T-Pose.
    h_flex_delta = human_angles['shoulder_flex'] - h_calib['shoulder_flex']
    h_abd_delta = human_angles['shoulder_abd'] - h_calib['shoulder_abd']
    h_elbow_delta = human_angles['elbow'] - h_calib['elbow']
    
    # Apply this deviation to the robot's reference pose, including scale factors for direction.
    target_shoulder_lift = r_calib['shoulder_lift_joint'] + math.radians(h_flex_delta * SCALE_FACTORS['shoulder_lift'])
    target_shoulder_pan = r_calib['shoulder_pan_joint'] + math.radians(h_abd_delta * SCALE_FACTORS['shoulder_pan'])
    target_elbow = r_calib['elbow_joint'] + math.radians(h_elbow_delta * SCALE_FACTORS['elbow'])

    # Map the wrist rotation similarly.
    target_wrist_1 = ROBOT_WRIST_1_AT_GRASP_REF_RAD
    if human_wrist_angle is not None and HUMAN_ANGLE_AT_GRASP_REF_DEG is not None:
        delta_deg = human_wrist_angle - HUMAN_ANGLE_AT_GRASP_REF_DEG
        # Apply the delta to the robot's reference wrist angle, with scaling.
        target_wrist_1 = ROBOT_WRIST_1_AT_GRASP_REF_RAD + math.radians(delta_deg * SCALE_FACTORS.get('wrist_1', 1.0))

    return [target_shoulder_pan, target_shoulder_lift, target_elbow, 
            target_wrist_1, ROBOT_WRIST_2_FIXED_RAD, ROBOT_WRIST_3_FIXED_RAD]


def translate_human_to_robot_joints(trajectory_df, T_camara_a_base):
    """
    Translates an entire human pose trajectory (from camera space) into a
    robot joint space trajectory. This is the core of the kinematic mapping pipeline.

    Args:
        trajectory_df (pd.DataFrame): DataFrame containing the raw human pose data for one phase.
        T_camara_a_base (np.array): The 4x4 transformation matrix from camera to robot base.

    Returns:
        np.array: A NumPy array of shape (N, 6) representing the robot's joint trajectory.
    """
    global HUMAN_ANGLE_AT_GRASP_REF_DEG
    robot_joint_trajectory = []
    
    # Establish the reference angle for the wrist rotation from the first valid frame of the phase.
    first_angle_val = trajectory_df['right_hand_rotation_angle_value'].dropna().iloc[0] if not trajectory_df['right_hand_rotation_angle_value'].dropna().empty else None
    if pd.notna(first_angle_val):
        HUMAN_ANGLE_AT_GRASP_REF_DEG = float(first_angle_val)
    else:
        HUMAN_ANGLE_AT_GRASP_REF_DEG = None

    # Process each frame of the human demonstration.
    for index, row in trajectory_df.iterrows():
        try:
            # 1. Load and transform all required keypoints from camera space to robot space.
            points_cam = {kp: np.array([row[f"{kp}_x"], row[f"{kp}_y"], row[f"{kp}_z"]]) for kp in KEY_POINTS_TO_PROCESS}
            points_robot = {kp: (T_camara_a_base @ np.append(p, 1))[:3] for kp, p in points_cam.items()}
            p_rs, p_re, p_rw = points_robot["right_shoulder"], points_robot["right_elbow"], points_robot["right_hand"]
            p_ls, p_rh = points_robot["left_shoulder"], points_robot["right_hip"]
            
            # 2. Apply "Morphological Reflection" to solve the "arm under the table" problem.
            # This mirrors the human's downward-hanging arm into an upward-reaching pose
            # that is suitable for the table-mounted robot.
            v_se = p_re - p_rs
            v_sw = p_rw - p_rs
            p_re[2] = p_rs[2] - v_se[2]
            p_rw[2] = p_rs[2] - v_sw[2]

            # 3. Calculate unambiguous human joint angles using a dynamic torso coordinate frame.
            # This frame is rebuilt for every frame using the torso landmarks.
            y_torso = p_rs - p_rh
            y_torso /= np.linalg.norm(y_torso)
            z_torso_temp = p_ls - p_rs
            x_torso = np.cross(y_torso, z_torso_temp)
            x_torso /= np.linalg.norm(x_torso)
            z_torso = np.cross(x_torso, y_torso)
            
            # Calculate angles relative to this new torso frame.
            v_humerus = p_re - p_rs
            v_forearm = p_rw - p_re
            angle_elbow = 180.0 - get_angle_between_vectors(-v_humerus, v_forearm)
            v_humerus_on_sagittal = np.array([np.dot(v_humerus, x_torso), np.dot(v_humerus, y_torso)])
            angle_shoulder_flex = math.degrees(math.atan2(v_humerus_on_sagittal[0], v_humerus_on_sagittal[1]))
            v_humerus_on_frontal = np.array([np.dot(v_humerus, y_torso), np.dot(v_humerus, z_torso)])
            angle_shoulder_abd = math.degrees(math.atan2(v_humerus_on_frontal[1], v_humerus_on_frontal[0]))
            
            current_human_angles = {'elbow': angle_elbow, 'shoulder_flex': angle_shoulder_flex, 'shoulder_abd': angle_shoulder_abd}
            
            # 4. Map the calculated human angles to the target robot joint angles.
            human_wrist_angle = row['right_hand_rotation_angle_value']
            robot_joints = map_human_to_robot_angles(current_human_angles, human_wrist_angle)
            robot_joint_trajectory.append(robot_joints)
            
        except Exception as e:
            # If a frame fails, append the previous frame's result to maintain trajectory length.
            if len(robot_joint_trajectory) > 0:
                robot_joint_trajectory.append(robot_joint_trajectory[-1])
            continue
            
    return np.array(robot_joint_trajectory)


def resample_trajectories(trajectory_list, num_points):
    """
    Resamples a list of trajectories to a fixed number of points using linear interpolation.
    
    This is a mandatory step before averaging or training DMPs, as all trajectories
    must have the same length.

    Args:
        trajectory_list (list): A list of NumPy arrays, where each array is a trajectory.
        num_points (int): The target number of points for each trajectory.

    Returns:
        list: A new list of resampled NumPy array trajectories.
    """
    resampled_trajectories = []
    for traj in trajectory_list:
        if len(traj) < 2:
            continue # Cannot interpolate with fewer than 2 points.
        original_indices = np.linspace(0, 1, len(traj))
        target_indices = np.linspace(0, 1, num_points)
        
        # Create a new array for the resampled trajectory.
        resampled_traj = np.zeros((num_points, traj.shape[1]))
        
        # Interpolate each dimension (joint) independently.
        for dim in range(traj.shape[1]):
            resampled_traj[:, dim] = np.interp(target_indices, original_indices, traj[:, dim])
        resampled_trajectories.append(resampled_traj)
        
    return resampled_trajectories


# ===== 4. Main Execution Block =====
if __name__ == '__main__':
    # --- Initialization ---
    try:
        T_base_a_camara = np.load(CALIBRATION_MATRIX_FILE)
        # We need the inverse transformation for mapping points from camera to robot base.
        T_camara_a_base = np.linalg.inv(T_base_a_camara)
        print("Camera-to-robot calibration matrix loaded successfully.")
    except FileNotFoundError:
        print(f"ERROR: Calibration file not found at '{CALIBRATION_MATRIX_FILE}'. Aborting."); sys.exit()

    all_demo_folders = find_all_demonstration_folders(DEMONSTRATIONS_BASE_DIR)
    if not all_demo_folders:
        print(f"ERROR: No demonstration folders found at '{DEMONSTRATIONS_BASE_DIR}'. Aborting."); sys.exit()
    print(f"Found {len(all_demo_folders)} demonstration folders to process.")

    # --- Main Learning Loop ---
    # Process each phase of the task separately to learn an independent DMP for each.
    for phase in PHASES_TO_LEARN:
        print("\n" + "="*50)
        print(f"LEARNING DMP FOR PHASE: '{phase}'")
        print("="*50)
        
        # The 'Rotating-Separated' phase might have a slightly different name pattern.
        phase_pattern = "Rotating-*" if phase == "Rotating-to-Separated" else phase
        
        all_robot_joint_trajectories = []

        # 1. Load and translate all demonstrations for the current phase.
        for demo_folder in all_demo_folders:
            human_demo_df = load_and_filter_phase_from_demo(demo_folder, phase_pattern)
            if human_demo_df is not None and not human_demo_df.empty:
                robot_joint_traj = translate_human_to_robot_joints(human_demo_df, T_camara_a_base)
                if len(robot_joint_traj) > 0:
                    all_robot_joint_trajectories.append(robot_joint_traj)
        
        if not all_robot_joint_trajectories:
            print(f"No valid demonstrations found for phase '{phase}'. Skipping.")
            continue
        print(f"Successfully translated {len(all_robot_joint_trajectories)} demonstrations for this phase.")
            
        # 2. Resample all translated trajectories to a consistent length.
        resampled_trajectories = resample_trajectories(all_robot_joint_trajectories, NUM_RESAMPLE_POINTS)
        if not resampled_trajectories:
            print(f"No valid trajectories remained after resampling for phase '{phase}'. Skipping.")
            continue
        print(f"Resampled all trajectories to {NUM_RESAMPLE_POINTS} points.")
        
        # 3. Calculate the average trajectory from all demonstrations.
        # The DMP will learn from this single, averaged trajectory, which represents the
        # "essential" motion of the task, with individual variations filtered out.
        averaged_trajectory = np.mean(np.array(resampled_trajectories), axis=0)
        print(f"Calculated average trajectory from {len(resampled_trajectories)} demonstrations.")

        # 4. Train the DMP model.
        # Initialize a DMP system with 6 dimensions (for the 6 robot joints).
        dmp = pydmps.dmp_discrete.DMPs_discrete(n_dmps=6, n_bfs=N_BASIS_FUNCTIONS)
        # Train the DMP to imitate the shape of the averaged trajectory.
        dmp.imitate_path(y_des=averaged_trajectory)
        print("DMP model trained successfully.")
        
        # 5. Save the learned model and its start/goal points to a file.
        # This .pkl file contains the complete, learned policy for this task phase.
        dmp_data_to_save = {
            'dmp_model': dmp,
            'start_pose': averaged_trajectory[0],
            'goal_pose': averaged_trajectory[-1]
        }
        dmp_filename = f"dmp_{phase.replace('-', '_').lower()}.pkl"
        dmp_output_path = os.path.join(DMP_OUTPUT_DIR, dmp_filename)
        
        try:
            with open(dmp_output_path, 'wb') as f:
                pickle.dump(dmp_data_to_save, f)
            print(f"DMP model saved to: {dmp_output_path}")
        except Exception as e:
            print(f"ERROR: Could not save DMP file: {e}")

    print("\n" + "="*50)
    print("DMP learning process for all phases has been completed.")
    print("="*50)