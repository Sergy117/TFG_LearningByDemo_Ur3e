#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Validation Script for Human-to-Robot Kinematic Mapping.

This script serves as a powerful debugging and validation tool. It loads a single
human demonstration, applies the full kinematic translation pipeline, and visualizes
the result in a 3D plot using Matplotlib.

The visualization displays two animated skeletons simultaneously:
1. The original human demonstration, transformed into the robot's coordinate frame.
2. The resulting robot arm pose, calculated by applying Forward Kinematics (FK) to the
   final translated joint angles.

This allows for a direct, frame-by-frame comparison to verify that the kinematic
mapping (including morphological reflection and relative angle mapping) is coherent
and produces logical robot movements.

Developed by: Sergio González Rodriguez (meanssergy@gmail.com)
For: University of Santiago de Compostela (USC) - Final Degree Project
Date: August 2025
"""

import numpy as np
import pandas as pd
import os
import glob
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time
import math
from scipy.signal import savgol_filter
import sys

# --- 1. Configuration ---

# File paths for the demonstration dataset and calibration files.
DEMONSTRATIONS_BASE_DIR = "/home/sergio/Escritorio/PracticasTFG/LBDOARABOHG/vision/action_segments_ALL3/action_segments_DEMO-*"
CALIBRATION_MATRIX_FILE = "/home/sergio/Escritorio/PracticasTFG/LBDOARABOHG/robot/data/T_base_a_camara.npy"
TASK_PHASE_SEQUENCE = ["Initial-Idle", "Grasping-to-Rotating", "Rotating-to-Separated", "Final-Final"]

# Key points required from the CSV file for the full kinematic mapping.
KEY_POINTS_TO_PROCESS = ["right_shoulder", "right_elbow", "right_hand", "left_shoulder", "right_hip"]

# Animation playback speed (delay in seconds between frames).
PLAYBACK_SPEED_DELAY = 0.05

# --- 2. Kinematic Mapping Configuration ---

# Reference poses for the human-robot mapping (based on the technical report, Sec. 3.2).
ROBOT_CALIB_ANGLES = {'shoulder_pan_joint': 0.0, 'shoulder_lift_joint': -math.pi/2, 'elbow_joint': 0.0}
HUMAN_CALIB_ANGLES_TPose = {'shoulder_flex': 0.0, 'shoulder_abd': 90.0, 'elbow': 180.0}
CALIBRATION_DATA = {'robot': ROBOT_CALIB_ANGLES, 'human': HUMAN_CALIB_ANGLES_TPose}

# Scale factors to invert joint directions, correcting for different kinematic conventions.
SCALE_FACTORS = {'shoulder_pan': -1.0, 'shoulder_lift': -1.0, 'elbow': -1.0}

# Default values for robot wrist joints.
HUMAN_ANGLE_AT_GRASP_REF_DEG = None
ROBOT_WRIST_1_AT_GRASP_REF_RAD, ROBOT_WRIST_2_FIXED_RAD, ROBOT_WRIST_3_FIXED_RAD = 0.0, 0.0, 0.0

# Standard UR3e Denavit-Hartenberg (DH) parameters for the Forward Kinematics visualization.
DH_PARAMS = {'d': np.array([0.1519,0,0,0.11235,0.08535,0.0819]), 'a': np.array([0,-0.24365,-0.21325,0,0,0]), 'alpha': np.array([math.pi/2,0,0,math.pi/2,-math.pi/2,0])}


# --- 3. Helper Functions ---

def find_all_demonstration_folders(base_dir_pattern):
    """Finds and numerically sorts all demonstration folders matching a pattern."""
    list_of_folders = glob.glob(base_dir_pattern)
    if not list_of_folders: return []
    def get_demo_number(p):
        try: return int(p.split('DEMO-')[1].split('_')[0])
        except: return -1
    return sorted(list_of_folders, key=get_demo_number)


def load_and_concat_demo(demo_folder_path, phase_sequence):
    """
    Loads and concatenates all CSV segments for a single demonstration, then cleans and filters the data.
    """
    df = None
    print(f"\nLoading: {os.path.basename(demo_folder_path)}")
    for phase in phase_sequence:
        phase_pattern = "Rotating-*" if phase == "Rotating-to-Separated" else phase
        files = sorted(glob.glob(os.path.join(demo_folder_path, f"segment_*_{phase_pattern}_*.csv")))
        if not files: print(f"  Warning: Could not find segment for '{phase}'."); continue
        try:
            s_df = pd.read_csv(files[0])
            print(f"  Loaded '{phase}' ({len(s_df)} frames).")
            if df is None: df = s_df
            else: df = pd.concat([df, s_df], ignore_index=True)
        except Exception as e: print(f"  Error loading {files[0]}: {e}")
    
    if df is not None:
        print(f"-> Total frames: {len(df)}.")
        # Clean and filter the combined trajectory data.
        for col in df.columns:
            if df[col].isnull().any(): df[col] = df[col].interpolate(method='linear', limit_direction='both')
            if len(df) > 11 and pd.api.types.is_numeric_dtype(df[col]):
                df[col] = savgol_filter(df[col], 11, 3)
        print("-> Data cleaned and filtered.")
    else:
        print("-> Could not load this demonstration.")
    return df


def get_angle_between_vectors(v1, v2):
    """Calculates the angle in degrees between two 3D vectors."""
    norm_v1, norm_v2 = np.linalg.norm(v1), np.linalg.norm(v2)
    if norm_v1 < 1e-6 or norm_v2 < 1e-6: return 0.0
    v1_u, v2_u = v1 / norm_v1, v2 / norm_v2
    return np.degrees(np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0)))


def forward_kinematics_ur3e(joint_angles):
    """
    Calculates the 3D positions of the UR3e's key links using Forward Kinematics.
    This is used for VISUALIZATION purposes only.
    """
    T = np.identity(4)
    link_positions = {'base': T[:3, 3]}
    thetas=joint_angles; d=DH_PARAMS['d']; a=DH_PARAMS['a']; alpha=DH_PARAMS['alpha']
    for i in range(6):
        T_i = np.array([[math.cos(thetas[i]),-math.sin(thetas[i])*math.cos(alpha[i]),math.sin(thetas[i])*math.sin(alpha[i]),a[i]*math.cos(thetas[i])],
                        [math.sin(thetas[i]),math.cos(thetas[i])*math.cos(alpha[i]),-math.cos(thetas[i])*math.sin(alpha[i]),a[i]*math.sin(thetas[i])],
                        [0,math.sin(alpha[i]),math.cos(alpha[i]),d[i]],
                        [0,0,0,1]])
        T = T @ T_i
        if i == 0: link_positions['shoulder'] = T[:3, 3] # After joint 1
        if i == 2: link_positions['elbow'] = T[:3, 3]    # After joint 3
        elif i == 5: link_positions['wrist'] = T[:3, 3] # After joint 6 (end-effector)
    return link_positions['base'], link_positions['shoulder'], link_positions['elbow'], link_positions['wrist']


def map_human_to_robot_angles(human_angles):
    """Applies the linear mapping with calibration offsets."""
    h_calib = CALIBRATION_DATA['human']; r_calib = CALIBRATION_DATA['robot']
    
    # Calculate human deviation from their reference T-Pose.
    h_flex_delta = human_angles['shoulder_flex'] - h_calib['shoulder_flex']
    h_abd_delta = human_angles['shoulder_abd'] - h_calib['shoulder_abd']
    h_elbow_delta = human_angles['elbow'] - h_calib['elbow']
    
    # Apply this deviation to the robot's reference pose.
    target_shoulder_lift = r_calib['shoulder_lift_joint'] + math.radians(h_flex_delta * SCALE_FACTORS['shoulder_lift'])
    target_shoulder_pan = r_calib['shoulder_pan_joint'] + math.radians(h_abd_delta * SCALE_FACTORS['shoulder_pan'])
    target_elbow = r_calib['elbow_joint'] + math.radians(h_elbow_delta * SCALE_FACTORS['elbow'])
    
    return [target_shoulder_pan, target_shoulder_lift, target_elbow]


def process_and_map_trajectory(trajectory_df, T_camara_a_base):
    """
    Processes the full human demonstration to generate both the transformed human pose
    and the final imitated robot pose for comparison.
    """
    global HUMAN_ANGLE_AT_GRASP_REF_DEG
    human_traj_points, robot_fk_traj = [], []
    
    # Establish a reference for the wrist rotation angle from the first valid frame.
    first_angle_val = trajectory_df['right_hand_rotation_angle_value'].dropna().iloc[0] if not trajectory_df['right_hand_rotation_angle_value'].dropna().empty else None
    if pd.notna(first_angle_val): HUMAN_ANGLE_AT_GRASP_REF_DEG = float(first_angle_val)
    else: HUMAN_ANGLE_AT_GRASP_REF_DEG = None; print("Warning: No valid grasp angle found for wrist rotation reference.")

    for index, row in trajectory_df.iterrows():
        # Load all required keypoints from the current row of the DataFrame.
        points_cam = {kp: np.array([row[f"{kp}_x"], row[f"{kp}_y"], row[f"{kp}_z"]]) for kp in KEY_POINTS_TO_PROCESS if f"{kp}_x" in row}
        if len(points_cam) != len(KEY_POINTS_TO_PROCESS):
            print(f"Warning: Missing keypoints in frame {index}. Skipping.")
            continue

        # 1. Transform all points from camera space to the robot's base frame.
        points_robot = {kp: (T_camara_a_base @ np.append(p, 1))[:3] for kp, p in points_cam.items()}
        p_rs, p_re, p_rw = points_robot["right_shoulder"], points_robot["right_elbow"], points_robot["right_hand"]
        p_ls, p_rh = points_robot["left_shoulder"], points_robot["right_hip"]
        
        # Store the transformed human pose for visualization.
        human_traj_points.append([p_rs, p_re, p_rw])
        
        # --- Full Translation Logic ---
        # 2. Apply Morphological Reflection to solve the "arm under the table" problem.
        v_se = p_re - p_rs; v_sw = p_rw - p_rs
        p_re[2] = p_rs[2] - v_se[2]; p_rw[2] = p_rs[2] - v_sw[2]

        # 3. Calculate unambiguous human joint angles using a dynamic torso frame.
        y_torso = p_rs - p_rh; y_torso /= np.linalg.norm(y_torso)
        z_torso_temp = p_ls - p_rs;
        x_torso = np.cross(y_torso, z_torso_temp); x_torso /= np.linalg.norm(x_torso)
        z_torso = np.cross(x_torso, y_torso)
        v_humerus = p_re - p_rs; v_forearm = p_rw - p_re
        angle_elbow = 180.0 - get_angle_between_vectors(-v_humerus, v_forearm)
        v_humerus_on_sagittal = np.array([np.dot(v_humerus, x_torso), np.dot(v_humerus, y_torso)])
        angle_shoulder_flex = math.degrees(math.atan2(v_humerus_on_sagittal[0], v_humerus_on_sagittal[1]))
        v_humerus_on_frontal = np.array([np.dot(v_humerus, y_torso), np.dot(v_humerus, z_torso)])
        angle_shoulder_abd = math.degrees(math.atan2(v_humerus_on_frontal[1], v_humerus_on_frontal[0]))
        current_human_angles = {'elbow': angle_elbow, 'shoulder_flex': angle_shoulder_flex, 'shoulder_abd': angle_shoulder_abd}

        # 4. Map human angles to the robot's main 3 arm joints.
        robot_main_joints = map_human_to_robot_angles(current_human_angles)
        
        # 5. Calculate the wrist rotation joint based on the human hand's rotation.
        human_angle = row['right_hand_rotation_angle_value'] if pd.notna(row['right_hand_rotation_angle_value']) else None
        wrist_1_angle_robot = ROBOT_WRIST_1_AT_GRASP_REF_RAD
        if human_angle is not None and HUMAN_ANGLE_AT_GRASP_REF_DEG is not None:
            delta_deg = human_angle - HUMAN_ANGLE_AT_GRASP_REF_DEG
            wrist_1_angle_robot = ROBOT_WRIST_1_AT_GRASP_REF_RAD + math.radians(delta_deg)
        
        # Combine all 6 joints.
        robot_joints = robot_main_joints + [wrist_1_angle_robot, ROBOT_WRIST_2_FIXED_RAD, ROBOT_WRIST_3_FIXED_RAD]
        
        # 6. Calculate the resulting robot pose using Forward Kinematics for visualization.
        b, s, e, w = forward_kinematics_ur3e(robot_joints)
        robot_fk_traj.append([b, s, e, w])
        
    return np.array(human_traj_points), np.array(robot_fk_traj)


def visualize_comparison(human_trajectory, robot_trajectory, demo_name):
    """
    Creates a 3D Matplotlib animation comparing the human and robot trajectories.
    """
    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection='3d')
    
    # Calculate plot bounds to fit both trajectories.
    all_points = np.vstack((human_trajectory.reshape(-1,3), robot_trajectory.reshape(-1,3)))
    min_b, max_b = np.min(all_points, axis=0) - 0.3, np.max(all_points, axis=0) + 0.3
    ax.set_xlim(min_b[0], max_b[0]); ax.set_ylim(min_b[1], max_b[1]); ax.set_zlim(min_b[2], max_b[2])
    ax.set_xlabel('X Axis (Robot Frame)'); ax.set_ylabel('Y Axis (Robot Frame)'); ax.set_zlabel('Z Axis (Robot Frame)'); ax.grid(True)
    ax.view_init(elev=25, azim=-120) # Set a consistent initial viewing angle.

    print("\n--- Starting Comparison Animation ---")
    for i in range(len(human_trajectory)):
        ax.cla() # Clear the plot for the next frame.
        # Re-apply settings after clearing.
        ax.set_xlim(min_b[0], max_b[0]); ax.set_ylim(min_b[1], max_b[1]); ax.set_zlim(min_b[2], max_b[2])
        ax.set_xlabel('X Axis (Robot Frame)'); ax.set_ylabel('Y Axis (Robot Frame)'); ax.set_zlabel('Z Axis (Robot Frame)'); ax.grid(True)
        ax.set_title(f"Comparing Demo: {demo_name} - Frame {i+1}/{len(human_trajectory)}")
        ax.view_init(elev=25, azim=-120)

        # Draw the human skeleton (blue).
        h_points = human_trajectory[i]
        ax.plot(h_points[:,0],h_points[:,1],h_points[:,2],'o-',color='dodgerblue',ms=8,lw=3,label='Human Demo (in Robot Frame)')
        ax.scatter(h_points[2,0], h_points[2,1], h_points[2,2], color='cyan', s=150, ec='black', depthshade=True)
        
        # Draw the robot skeleton (green).
        r_points = robot_trajectory[i]
        ax.plot(r_points[:,0],r_points[:,1],r_points[:,2],'o-',color='limegreen',ms=8,lw=3,label='Robot Imitation (FK)')
        ax.scatter(r_points[3,0], r_points[3,1], r_points[3,2], color='lime', s=150, ec='black', depthshade=True)
        
        if i == 0: ax.legend() # Show legend only on the first frame.
        plt.pause(PLAYBACK_SPEED_DELAY)
        
    print("--- Comparison Finished ---")
    plt.show() # Keep the final plot open.


# ===== MAIN EXECUTION BLOCK =====
if __name__ == '__main__':
    # --- Initialization ---
    try:
        T_base_a_camara = np.load(CALIBRATION_MATRIX_FILE)
        T_camara_a_base = np.linalg.inv(T_base_a_camara)
        print("Calibration matrix loaded.")
    except FileNotFoundError:
        print(f"ERROR: Calibration file not found at '{CALIBRATION_MATRIX_FILE}'."); sys.exit()

    all_demos = find_all_demonstration_folders(DEMONSTRATIONS_BASE_DIR)
    if not all_demos:
        print(f"No demonstration folders found matching pattern: {DEMONSTRATIONS_BASE_DIR}")
    else:
        print(f"Found {len(all_demos)} demonstrations to validate.")
        current_demo_index = 0
        
        # Interactive loop to cycle through demonstrations.
        while current_demo_index < len(all_demos):
            demo_folder = all_demos[current_demo_index]
            demo_name = os.path.basename(demo_folder)
            
            # 1. Load the raw demonstration data.
            raw_trajectory_df = load_and_concat_demo(demo_folder, TASK_PHASE_SEQUENCE)
            
            if raw_trajectory_df is not None:
                # 2. Process the data to get both human and robot trajectories.
                human_tf_traj, robot_fk_traj = process_and_map_trajectory(raw_trajectory_df, T_camara_a_base)
                
                # 3. Visualize the comparison if processing was successful.
                if len(human_tf_traj) > 0:
                    visualize_comparison(human_tf_traj, robot_fk_traj, demo_name)
                else:
                    print("Could not process trajectory for visualization.")

            # 4. User menu for navigation.
            while True:
                user_input = input(f"\nDemo '{demo_name}' validated. Choose action:\n (n) Next Demo\n (r) Replay\n (q) Quit\n> ").lower()
                if user_input == 'n':
                    current_demo_index += 1; break
                elif user_input == 'r':
                    break
                elif user_input == 'q':
                    current_demo_index = float('inf'); break
                else:
                    print("Invalid option.")
                    
    print("\nValidation process finished.")