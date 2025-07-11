#!/usr/bin/env python3

import rospy
import moveit_commander
import sys
import time
import math
import numpy as np
import pandas as pd
import os
import glob
from ur_msgs.srv import SetIO # Asegúrate de tener esto

# --- Configuración ---
ACTION_SEGMENTS_BASE_DIR = "/home/robotica/ws_moveit/src/ur3e_control/data/action_segments_20250604_151108" # Ajusta
HOME_JOINTS = [0.00455, -1.57309, 0.01101, -1.57567, 0.00152, -0.00622]

TASK_PHASE_SEQUENCE = [
    "Initial-Idle",
    "Grasping-to-Rotating",
    "Rotating-Separated"
]

GRIPPER_PIN = 0
GRIPPER_STATE_OPEN = 0.0
GRIPPER_STATE_CLOSE = 1.0

# --- NUEVO: Offset Global Manual y Parámetros de Transformación Morfológica ---
# 1. Primero, obtén las coordenadas de la CABEZA de tu muñeca en el frame de la CÁMARA
#    cuando está en tu "posición de inicio de demostración".
CABEZA_CAM_REF = np.array([-0.3728, -0.0388, 0.6640])
ROBOT_TARGET_FOR_HEAD_REF = np.array([0.2131392980726331, -0.14994444363697007, 0.20789546224132807])

MANUAL_GLOBAL_OFFSET = ROBOT_TARGET_FOR_HEAD_REF - CABEZA_CAM_REF
#MANUAL_GLOBAL_OFFSET = np.array([0.585939, -0.111144, -0.456105]) # <-- ¡TUS VALORES CALCULADOS!

# Para el "volteo" morfológico, es mejor ajustarlo en la función de cálculo de joints.
# Pero si quisieras una inversión Z simple de los vectores relativos al hombro:
INVERT_Z_FOR_MORPHOLOGY = True # True para intentar "subir" el brazo

# --- Variables de Orientación (como antes) ---
HUMAN_ANGLE_AT_GRASP_REF_DEG = None
ROBOT_WRIST_1_AT_GRASP_REF_RAD = 0.0
ROBOT_WRIST_2_FIXED_RAD = 0.0 # Ajustar para agarre horizontal (ej. 0 o math.pi/2)
ROBOT_WRIST_3_FIXED_RAD = 0.0

# --- Funciones (control_gripper, etc. - Asegúrate que calculate_joint_angles_for_robot está aquí) ---
# (Omito las funciones que no cambian para brevedad, pero deben estar)
def control_gripper(pin, state):
    rospy.loginfo(f"Intentando establecer pin {pin} a {state} para el gripper...")
    try:
        rospy.wait_for_service('/ur_hardware_interface/set_io', timeout=2.0)
    except rospy.ROSException:
        rospy.logerr("Servicio /ur_hardware_interface/set_io no disponible.")
        return False
    try:
        set_io = rospy.ServiceProxy('/ur_hardware_interface/set_io', SetIO)
        resp = set_io(fun=1, pin=pin, state=float(state))
        if resp.success: rospy.loginfo(f"GRIPPER: Pin {pin} en {state}")
        else: rospy.logwarn(f"GRIPPER: Fallo al establecer pin {pin} en {state}")
        return resp.success
    except rospy.ServiceException as e:
        rospy.logerr(f"GRIPPER: Error IO: {e}")
        return False

def calculate_joint_angles_for_robot(shoulder_3d, elbow_3d, wrist_3d,
                                     human_current_wrist_angle_deg=None):
    global HUMAN_ANGLE_AT_GRASP_REF_DEG, ROBOT_WRIST_1_AT_GRASP_REF_RAD
    global ROBOT_WRIST_2_FIXED_RAD, ROBOT_WRIST_3_FIXED_RAD

    v_upper_arm = elbow_3d - shoulder_3d
    v_forearm = wrist_3d - elbow_3d

    # --- Ajustes de Signos para Morfología (EXPERIMENTALES) ---
    # Si el robot sigue yendo "debajo de la mesa", aquí es donde hay que jugar.
    # La idea es que si el hombro humano está arriba y el codo abajo (v_upper_arm[2] < 0),
    # el robot necesita un shoulder_lift que vaya "hacia arriba" desde su base.

    shoulder_pan_angle = math.atan2(v_upper_arm[1], v_upper_arm[0])
    
    len_v_upper_arm_xy = math.sqrt(v_upper_arm[0]**2 + v_upper_arm[1]**2)
    if len_v_upper_arm_xy < 1e-6 :
        # Brazo apuntando casi verticalmente respecto a la base
        shoulder_lift_angle = math.pi/2 if -v_upper_arm[2] > 0 else -math.pi/2
    else:
        # El -v_upper_arm[2] ya intenta una inversión. Si no funciona, prueba v_upper_arm[2]
        # o multiplica el ángulo resultante por -1.
        shoulder_lift_angle = math.atan2(-v_upper_arm[2], len_v_upper_arm_xy)

    norm_v_upper_arm = np.linalg.norm(v_upper_arm)
    norm_v_forearm = np.linalg.norm(v_forearm)
    if norm_v_upper_arm < 1e-6 or norm_v_forearm < 1e-6:
        elbow_angle = 0.0 # Brazo o antebrazo de longitud cero
    else:
        dot_product = np.dot(-v_upper_arm, v_forearm)
        elbow_angle_rad = math.acos(np.clip(dot_product / (norm_v_upper_arm * norm_v_forearm), -1.0, 1.0))
        elbow_angle = elbow_angle_rad - math.pi # 0 es recto
        # Considera si el signo del elbow_angle necesita invertirse para la convención del UR3e
        # ej. elbow_angle *= -1

    wrist_1_angle_robot = ROBOT_WRIST_1_AT_GRASP_REF_RAD
    if human_current_wrist_angle_deg is not None and HUMAN_ANGLE_AT_GRASP_REF_DEG is not None:
        human_wrist_rotation_delta_deg = human_current_wrist_angle_deg - HUMAN_ANGLE_AT_GRASP_REF_DEG
        robot_wrist_rotation_delta_rad = math.radians(human_wrist_rotation_delta_deg)
        wrist_1_angle_robot = ROBOT_WRIST_1_AT_GRASP_REF_RAD + robot_wrist_rotation_delta_rad
    
    wrist_2_angle_robot = ROBOT_WRIST_2_FIXED_RAD
    wrist_3_angle_robot = ROBOT_WRIST_3_FIXED_RAD

    joint_angles = [
        shoulder_pan_angle, shoulder_lift_angle, elbow_angle,
        wrist_1_angle_robot, wrist_2_angle_robot, wrist_3_angle_robot
    ]
    return joint_angles

def go_to_joint_state(move_group, joints, wait=True):
    move_group.set_joint_value_target(joints)
    plan_success, plan, _, _ = move_group.plan()
    if plan_success:
        move_group.execute(plan, wait=wait)
        return True
    else:
        rospy.logerr("No se encontró plan articular válido.")
        return False

def find_latest_segment_folder(base_dir_pattern):
    list_of_folders = glob.glob(base_dir_pattern)
    if not list_of_folders: return None
    return max(list_of_folders, key=os.path.getctime)

def load_trajectory_from_csv(folder_path, phase_name_pattern):
    search_pattern = os.path.join(folder_path, f"segment_*_{phase_name_pattern}_*.csv")
    matching_files = glob.glob(search_pattern)
    if not matching_files:
        rospy.logwarn(f"No CSV para patrón '{phase_name_pattern}' en '{folder_path}'")
        return None
    trajectory_file = sorted(matching_files)[0]
    rospy.loginfo(f"Cargando trayectoria desde: {trajectory_file}")
    try:
        df = pd.read_csv(trajectory_file)
        pos_cols_base = ["right_shoulder", "right_elbow", "right_hand"]
        pos_cols_full = []
        for base in pos_cols_base: pos_cols_full.extend([f"{base}_x", f"{base}_y", f"{base}_z"])
        angle_col = "right_hand_rotation_angle_value"
        expected_cols = pos_cols_full + [angle_col]
        if not all(col in df.columns for col in expected_cols):
            missing = [col for col in expected_cols if col not in df.columns]
            rospy.logerr(f"CSV {trajectory_file} no tiene columnas esperadas. Faltan: {missing}. Encontradas: {df.columns.tolist()}")
            return None
        return df
    except Exception as e:
        rospy.logerr(f"Error al cargar CSV {trajectory_file}: {e}")
        return None
# ===== MAIN =====
if __name__ == '__main__':
    try:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('ur3e_manual_offset_player', anonymous=True)

        group_name = "manipulator"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        # ... (configuración de move_group) ...

        rospy.loginfo("Robot inicializado. Yendo a HOME.")
        # ... (go_to_joint_state(move_group, HOME_JOINTS)) ...
        time.sleep(1)

        latest_segments_folder = find_latest_segment_folder(ACTION_SEGMENTS_BASE_DIR)
        if not latest_segments_folder:
            rospy.logerr(f"No se encontraron carpetas de segmentos. Abortando."); sys.exit()
        rospy.loginfo(f"Usando carpeta de segmentos: {latest_segments_folder}")

        previous_phase_completed = ""

        for phase_idx, target_phase in enumerate(TASK_PHASE_SEQUENCE):
            if rospy.is_shutdown(): break
            rospy.loginfo(f"\n--- Iniciando Fase: {target_phase} ---")

            phase_pattern_to_load = target_phase
            if target_phase == "Rotating-Separated": phase_pattern_to_load = "Rotating-*"
            
            trajectory_df = load_trajectory_from_csv(latest_segments_folder, phase_pattern_to_load)

            if trajectory_df is not None and not trajectory_df.empty:
                rospy.loginfo(f"Procesando {len(trajectory_df)} puntos para '{target_phase}'...")

                # Lógica de Gripper y Referencia de Ángulo
                if target_phase == "Initial-Idle":
                    control_gripper(GRIPPER_PIN, GRIPPER_STATE_OPEN); time.sleep(0.5)
                
                if target_phase == "Grasping-to-Rotating":
                    first_angle_val = trajectory_df.iloc[0]['right_hand_rotation_angle_value']
                    if pd.notna(first_angle_val):
                        HUMAN_ANGLE_AT_GRASP_REF_DEG = float(first_angle_val)
                        rospy.loginfo(f"Establecida Referencia Ángulo Humano Agarre: {HUMAN_ANGLE_AT_GRASP_REF_DEG:.2f} grados")
                    else:
                        rospy.logwarn("Primer ángulo en Grasping-to-Rotating es NaN/None. No se pudo establecer referencia de agarre.")
                
                if previous_phase_completed == "Grasping-to-Rotating" and target_phase == "Rotating-Separated":
                    rospy.loginfo("Acción: CERRAR GRIPPER")
                    control_gripper(GRIPPER_PIN, GRIPPER_STATE_CLOSE); time.sleep(1)

                for index, row in trajectory_df.iterrows():
                    if rospy.is_shutdown(): break
                    shoulder_cam_raw = np.array([row['right_shoulder_x'], row['right_shoulder_y'], row['right_shoulder_z']])
                    elbow_cam_raw = np.array([row['right_elbow_x'], row['right_elbow_y'], row['right_elbow_z']])
                    wrist_cam_raw = np.array([row['right_hand_x'], row['right_hand_y'], row['right_hand_z']])
                    
                    if np.isnan(shoulder_cam_raw).any() or np.isnan(elbow_cam_raw).any() or np.isnan(wrist_cam_raw).any():
                        rospy.logwarn(f"Datos NaN en CSV frame {index}. Saltando."); continue
                        continue
                    # --- APLICAR OFFSET GLOBAL MANUAL ---
                    shoulder_robot = shoulder_cam_raw + MANUAL_GLOBAL_OFFSET
                    elbow_robot    = elbow_cam_raw    + MANUAL_GLOBAL_OFFSET
                    wrist_robot    = wrist_cam_raw    + MANUAL_GLOBAL_OFFSET
                    # --- FIN OFFSET ---
                                        
                    current_human_angle_deg = None
                    if pd.notna(row['right_hand_rotation_angle_value']):
                        current_human_angle_deg = float(row['right_hand_rotation_angle_value'])
                    
                    target_joint_angles = calculate_joint_angles_for_robot(
                        shoulder_robot, elbow_robot, wrist_robot,
                        human_current_wrist_angle_deg=current_human_angle_deg
                    )
                    
                    if not go_to_joint_state(move_group, target_joint_angles, wait=True):
                        rospy.logwarn(f"Fallo al planificar/ejecutar para punto {index} de fase '{target_phase}'.")
                
                rospy.loginfo(f"Fase '{target_phase}' completada.")
                previous_phase_completed = target_phase

                if target_phase == "Rotating-Separated":
                    rospy.loginfo("Acción: ABRIR GRIPPER")
                    control_gripper(GRIPPER_PIN, GRIPPER_STATE_OPEN); time.sleep(1)
            
            else: rospy.logwarn(f"No se cargó trayectoria para '{target_phase}'. Saltando.")
            
            if not rospy.is_shutdown() and phase_idx < len(TASK_PHASE_SEQUENCE) - 1:
                rospy.loginfo("Pausa antes de la siguiente fase..."); time.sleep(0.5)

        rospy.loginfo("\n--- Secuencia de Tarea Completa Finalizada ---")
        rospy.loginfo("Volviendo a HOME.")
        go_to_joint_state(move_group, HOME_JOINTS)

    except rospy.ROSInterruptException: rospy.loginfo("Programa interrumpido.")
    except Exception as e: rospy.logerr(f"Excepción no controlada: {e}"); import traceback; traceback.print_exc()
    finally: rospy.loginfo("Finalizando."); moveit_commander.roscpp_shutdown()