#!/usr/bin/env python3

import numpy as np
import pandas as pd
import os
import glob
import math
import pickle
from scipy.signal import savgol_filter
import pydmps.dmp_discrete
import sys

# --- Configuración ---
DEMONSTRATIONS_BASE_DIR = "/home/sergio/Escritorio/PracticasTFG/LBDOARABOHG/vision/action_segments_ALL3/action_segments_DEMO-*"
CALIBRATION_MATRIX_FILE = "TFG_Sergio/auxiliary_tools/cam_cal/data/T_base_a_camara.npy"
PHASES_TO_LEARN = ["Initial-Idle", "Grasping-to-Rotating", "Rotating-Separated", "Final-Final"]
DMP_OUTPUT_DIR = "./learned_dmps_final2" # Guardar en una nueva carpeta para no mezclar con los antiguos
os.makedirs(DMP_OUTPUT_DIR, exist_ok=True)

# --- Parámetros de Preprocesamiento y DMP ---
NUM_RESAMPLE_POINTS = 200
N_BASIS_FUNCTIONS = 30

# --- Configuración del Mapeo Cinemático ---
KEY_POINTS_TO_PROCESS = ["right_shoulder", "right_elbow", "right_hand", "left_shoulder", "right_hip"]
ROBOT_CALIB_ANGLES = {'shoulder_pan_joint': 0.0, 'shoulder_lift_joint': -math.pi/2, 'elbow_joint': 0.0}
HUMAN_CALIB_ANGLES_TPose = {'shoulder_flex': 0.0, 'shoulder_abd': 90.0, 'elbow': 180.0}
CALIBRATION_DATA = {'robot': ROBOT_CALIB_ANGLES, 'human': HUMAN_CALIB_ANGLES_TPose}
SCALE_FACTORS = {'shoulder_pan': -1.0, 'shoulder_lift': -1.0, 'elbow': -1.0, 'wrist_1': -1.0} # Añadido wrist_1 por si necesita invertirse
ROBOT_WRIST_1_AT_GRASP_REF_RAD, ROBOT_WRIST_2_FIXED_RAD, ROBOT_WRIST_3_FIXED_RAD = 0.0, 0.0, 0.0
HUMAN_ANGLE_AT_GRASP_REF_DEG = None

# --- Funciones ---
def find_all_demonstration_folders(base_dir_pattern):
    list_of_folders = glob.glob(base_dir_pattern);
    if not list_of_folders: return []
    def get_demo_number(p):
        try: return int(p.split('DEMO-')[1].split('_')[0])
        except: return -1
    return sorted(list_of_folders, key=get_demo_number)

def load_and_filter_phase_from_demo(demo_folder_path, phase_name_pattern):
    search_pattern = os.path.join(demo_folder_path, f"segment_*_{phase_name_pattern}_*.csv")
    matching_files = sorted(glob.glob(search_pattern))
    if not matching_files:
        # No imprimir advertencia aquí, es normal que no todas las demos tengan todas las fases
        return None
    try:
        df = pd.read_csv(matching_files[0])
        for col in df.columns:
            if df[col].isnull().any(): df[col] = df[col].interpolate(method='linear', limit_direction='both')
        df = df.dropna()
        if len(df) <= 11: return None
        for col in df.columns:
            if pd.api.types.is_numeric_dtype(df[col]):
                df[col] = savgol_filter(df[col], 11, 3)
        return df
    except Exception as e:
        print(f"  -> Error al cargar o filtrar {matching_files[0]}: {e}")
        return None

def get_angle_between_vectors(v1, v2):
    norm_v1, norm_v2 = np.linalg.norm(v1), np.linalg.norm(v2)
    if norm_v1 < 1e-6 or norm_v2 < 1e-6: return 0.0
    v1_u, v2_u = v1 / norm_v1, v2 / norm_v2
    return np.degrees(np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0)))

def map_human_to_robot_angles(human_angles, human_wrist_angle):
    global HUMAN_ANGLE_AT_GRASP_REF_DEG
    h_calib = CALIBRATION_DATA['human']; r_calib = CALIBRATION_DATA['robot']
    
    h_flex_delta = human_angles['shoulder_flex'] - h_calib['shoulder_flex']
    h_abd_delta = human_angles['shoulder_abd'] - h_calib['shoulder_abd']
    h_elbow_delta = human_angles['elbow'] - h_calib['elbow']
    
    target_shoulder_lift = r_calib['shoulder_lift_joint'] + math.radians(h_flex_delta * SCALE_FACTORS['shoulder_lift'])
    target_shoulder_pan = r_calib['shoulder_pan_joint'] + math.radians(h_abd_delta * SCALE_FACTORS['shoulder_pan'])
    target_elbow = r_calib['elbow_joint'] + math.radians(h_elbow_delta * SCALE_FACTORS['elbow'])

    target_wrist_1 = ROBOT_WRIST_1_AT_GRASP_REF_RAD
    if human_wrist_angle is not None and HUMAN_ANGLE_AT_GRASP_REF_DEG is not None:
        delta_deg = human_wrist_angle - HUMAN_ANGLE_AT_GRASP_REF_DEG
        target_wrist_1 = ROBOT_WRIST_1_AT_GRASP_REF_RAD + math.radians(delta_deg * SCALE_FACTORS.get('wrist_1', 1.0))

    return [target_shoulder_pan, target_shoulder_lift, target_elbow, 
            target_wrist_1, ROBOT_WRIST_2_FIXED_RAD, ROBOT_WRIST_3_FIXED_RAD]

def translate_human_to_robot_joints(trajectory_df, T_camara_a_base):
    global HUMAN_ANGLE_AT_GRASP_REF_DEG
    robot_joint_trajectory = []
    
    first_angle_val = trajectory_df['right_hand_rotation_angle_value'].dropna().iloc[0] if not trajectory_df['right_hand_rotation_angle_value'].dropna().empty else None
    if pd.notna(first_angle_val): HUMAN_ANGLE_AT_GRASP_REF_DEG = float(first_angle_val)
    else: HUMAN_ANGLE_AT_GRASP_REF_DEG = None

    for index, row in trajectory_df.iterrows():
        try:
            points_cam = {kp: np.array([row[f"{kp}_x"], row[f"{kp}_y"], row[f"{kp}_z"]]) for kp in KEY_POINTS_TO_PROCESS}
            points_robot = {kp: (T_camara_a_base @ np.append(p, 1))[:3] for kp, p in points_cam.items()}
            p_rs, p_re, p_rw = points_robot["right_shoulder"], points_robot["right_elbow"], points_robot["right_hand"]
            p_ls, p_rh = points_robot["left_shoulder"], points_robot["right_hip"]
            
            v_se = p_re - p_rs; v_sw = p_rw - p_rs
            p_re[2] = p_rs[2] - v_se[2]; p_rw[2] = p_rs[2] - v_sw[2]

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
            
            human_wrist_angle = row['right_hand_rotation_angle_value']
            robot_joints = map_human_to_robot_angles(current_human_angles, human_wrist_angle)
            robot_joint_trajectory.append(robot_joints)
        except Exception as e:
            if len(robot_joint_trajectory) > 0: robot_joint_trajectory.append(robot_joint_trajectory[-1])
            continue
            
    return np.array(robot_joint_trajectory)

def resample_trajectories(trajectory_list, num_points):
    resampled_trajectories = []
    for traj in trajectory_list:
        if len(traj) < 2: continue
        original_indices = np.linspace(0, 1, len(traj))
        target_indices = np.linspace(0, 1, num_points)
        resampled_traj = np.zeros((num_points, traj.shape[1]))
        for dim in range(traj.shape[1]):
            resampled_traj[:, dim] = np.interp(target_indices, original_indices, traj[:, dim])
        resampled_trajectories.append(resampled_traj)
    return resampled_trajectories

# ===== MAIN =====
if __name__ == '__main__':
    try:
        T_base_a_camara = np.load(CALIBRATION_MATRIX_FILE)
        T_camara_a_base = np.linalg.inv(T_base_a_camara)
        print("Matriz de calibración cámara-robot cargada.")
    except FileNotFoundError:
        print(f"ERROR: No se encontró '{CALIBRATION_MATRIX_FILE}'. Abortando."); sys.exit()

    all_demo_folders = find_all_demonstration_folders(DEMONSTRATIONS_BASE_DIR)
    if not all_demo_folders:
        print(f"No se encontraron demos en: {DEMONSTRATIONS_BASE_DIR}"); sys.exit()
    print(f"Encontradas {len(all_demo_folders)} carpetas de demostración para procesar.")

    for phase in PHASES_TO_LEARN:
        print("\n" + "="*50)
        print(f"APRENDIENDO DMP PARA LA FASE: '{phase}'")
        print("="*50)
        
        phase_pattern = "Rotating-*" if phase == "Rotating-to-Separated" else phase
        all_robot_joint_trajectories = []

        for demo_folder in all_demo_folders:
            human_demo_df = load_and_filter_phase_from_demo(demo_folder, phase_pattern)
            if human_demo_df is not None and not human_demo_df.empty:
                robot_joint_traj = translate_human_to_robot_joints(human_demo_df, T_camara_a_base)
                if len(robot_joint_traj) > 0:
                    all_robot_joint_trajectories.append(robot_joint_traj)
        
        if not all_robot_joint_trajectories:
            print(f"No hay datos válidos para aprender para la fase '{phase}'. Saltando.")
            continue
        print(f"Se tradujeron {len(all_robot_joint_trajectories)} demostraciones para esta fase.")
            
        resampled_trajectories = resample_trajectories(all_robot_joint_trajectories, NUM_RESAMPLE_POINTS)
        if not resampled_trajectories:
            print(f"No quedaron trayectorias válidas después del re-muestreo para la fase '{phase}'. Saltando.")
            continue
        print(f"Trayectorias re-muestreadas a {NUM_RESAMPLE_POINTS} puntos.")
        
        averaged_trajectory = np.mean(np.array(resampled_trajectories), axis=0)
        print(f"Calculada trayectoria promedio de {len(resampled_trajectories)} demostraciones.")

        dmp = pydmps.dmp_discrete.DMPs_discrete(n_dmps=6, n_bfs=N_BASIS_FUNCTIONS)
        dmp.imitate_path(y_des=averaged_trajectory)
        print("DMP entrenado exitosamente.")
        
        dmp_data_to_save = {'dmp_model': dmp, 'start_pose': averaged_trajectory[0], 'goal_pose': averaged_trajectory[-1]}
        dmp_filename = f"dmp_{phase.replace('-', '_').lower()}.pkl"
        dmp_output_path = os.path.join(DMP_OUTPUT_DIR, dmp_filename)
        
        try:
            with open(dmp_output_path, 'wb') as f: pickle.dump(dmp_data_to_save, f)
            print(f"Modelo DMP guardado en: {dmp_output_path}")
        except Exception as e:
            print(f"ERROR al guardar el archivo DMP: {e}")

    print("\nProceso de aprendizaje de todos los DMPs finalizado.")