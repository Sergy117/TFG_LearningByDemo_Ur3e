#!/usr/bin/env python3

import numpy as np
import cv2
import os
import glob
import pandas as pd
from scipy.spatial.transform import Rotation as R_scipy
import sys
# --- Configuración ---
HAND_EYE_IMAGES_DIR = "./hand_eye_images/cam1" 
ROBOT_POSES_FILE = "./hand_eye_data/robot_poses.csv" 
CAMERA_MATRIX_FILE = "data/camera_matrix_cam1.npy"
DIST_COEFFS_FILE = "data/dist_coeffs_cam1.npy"
CALIB_OUTPUT_DIR = "data"
chessboard_size = (9, 6)
square_size = 0.025

if __name__ == '__main__':
    print("Iniciando calibración Cámara-Robot (Hand-Eye)...")

    # 1. Cargar parámetros intrínsecos y datos capturados
    camera_matrix = np.load(CAMERA_MATRIX_FILE)
    dist_coeffs = np.load(DIST_COEFFS_FILE)
    robot_poses_df = pd.read_csv(ROBOT_POSES_FILE)
    
    # 2. Preparar puntos del objeto (3D) para el tablero
    objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
    objp *= square_size

    # Listas para almacenar las transformaciones para calibrateHandEye
    R_gripper2base_list, t_gripper2base_list = [], []
    R_target2cam_list, t_target2cam_list = [], []

    print(f"Procesando {len(robot_poses_df)} poses...")
    for index, row in robot_poses_df.iterrows():
        image_path = os.path.join(HAND_EYE_IMAGES_DIR, row['image_name'])
        image = cv2.imread(image_path)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)
        if ret:
            # --- Pose del Tablero en la Cámara (T_camara_a_patron) ---
            corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                                              (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
            ret_pnp, rvec, tvec = cv2.solvePnP(objp, corners_refined, camera_matrix, dist_coeffs)
            if not ret_pnp: continue
            
            R_target2cam, _ = cv2.Rodrigues(rvec)
            t_target2cam = tvec
            
            # --- Pose del Robot (T_base_a_herramienta) ---
            pos = np.array([row['pos_x'], row['pos_y'], row['pos_z']])
            quat = np.array([row['quat_x'], row['quat_y'], row['quat_z'], row['quat_w']])
            
            R_base2gripper = R_scipy.from_quat(quat).as_matrix()
            t_base2gripper = pos.reshape(3, 1)

            # OpenCV necesita la transformación inversa (gripper a base)
            R_gripper2base = R_base2gripper.T
            t_gripper2base = -R_gripper2base @ t_base2gripper

            R_gripper2base_list.append(R_gripper2base)
            t_gripper2base_list.append(t_gripper2base)
            R_target2cam_list.append(R_target2cam)
            t_target2cam_list.append(t_target2cam)
            print(f"  Pose {index+1} procesada correctamente.")
        else:
            print(f"  No se encontró tablero en '{row['image_name']}'. Saltando pose.")

    if len(R_gripper2base_list) < 5:
        print(f"ERROR: No hay suficientes poses válidas para la calibración (se encontraron {len(R_gripper2base_list)}).")
        sys.exit()
    
    print(f"\nRealizando calibración Hand-Eye con {len(R_gripper2base_list)} poses.")
    
    # Esta función calcula la pose de la cámara respecto a la base del robot (para una cámara fija)
    R_base2cam, t_base2cam = cv2.calibrateHandEye(
        R_gripper2base_list, t_gripper2base_list,
        R_target2cam_list, t_target2cam_list,
        method=cv2.CALIB_HAND_EYE_PARK # PARK y TSAI son métodos comunes
    )

    # --- Resultados ---
    T_base_a_camara = np.identity(4)
    T_base_a_camara[:3, :3] = R_base2cam
    T_base_a_camara[:3, 3] = t_base2cam.flatten()

    print("\n--- ¡Calibración Cámara-Robot Completada! ---")
    print("Matriz de Transformación Homogénea (T_base_a_camara):")
    print(np.round(T_base_a_camara, 4))

    if not os.path.exists(CALIB_OUTPUT_DIR): os.makedirs(CALIB_OUTPUT_DIR)
    output_file = os.path.join(CALIB_OUTPUT_DIR, "T_base_a_camara.npy")
    np.save(output_file, T_base_a_camara)
    print(f"\nMatriz de transformación guardada en: '{output_file}'")