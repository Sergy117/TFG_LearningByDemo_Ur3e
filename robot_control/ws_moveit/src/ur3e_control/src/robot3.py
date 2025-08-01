#!/usr/bin/env python3

import rospy
import moveit_commander
import sys
import time
import math
import numpy as np
import pandas as pd # Para leer CSVs
import os
import glob       # Para encontrar archivos
from ur_msgs.srv import SetIO # Asegúrate de tener este tipo de mensaje si usas /ur_hardware_interface/set_io

# --- Configuración ---
ACTION_SEGMENTS_BASE_DIR = "/home/robotica/ws_moveit/src/ur3e_control/data/action_segments_*"
Z_OFFSET = -1.0
X_OFFSET = 0.0
Y_OFFSET = 0.0
GLOBAL_OFFSET = np.array([X_OFFSET, Y_OFFSET, Z_OFFSET])
HOME_JOINTS = [0.00455, -1.57309, 0.01101, -1.57567, 0.00152, -0.00622]

# --- NUEVO: Secuencia de Fases de la Tarea Completa ---
# Define el orden de las fases tal como se guardan en los nombres de archivo
TASK_PHASE_SEQUENCE = [
    "Initial-Idle",             # Aproximación
    "Grasping-to-Rotating",     # Agarre hasta inicio de rotación
    "Rotating-Separated"        # Rotación hasta separación (o "Rotating-HandMovedAway")
    # "Final-Idle"            # Opcional: si quieres un movimiento final de vuelta
]

# --- NUEVO: Configuración del Gripper (Ejemplo) ---
# Estos son ejemplos, ajústalos a tu hardware y configuración de IO del UR
GRIPPER_PIN = 0  # Pin digital para controlar el gripper (ej. DO0)
GRIPPER_STATE_OPEN = 0.0
GRIPPER_STATE_CLOSE = 1.0

# --- Funciones (control_gripper, calculate_joint_angles_from_3d_points, go_to_joint_state, find_latest_segment_folder, load_trajectory_from_csv) ---
# (Estas funciones permanecen IGUAL que en la respuesta anterior, las omito aquí por brevedad pero deben estar)
def control_gripper(pin, state):
    """Controla el gripper (requiere que el servicio IO esté configurado)."""
    rospy.loginfo(f"Intentando establecer pin {pin} a {state} para el gripper...")
    # Verificar si el servicio está disponible con un timeout
    try:
        rospy.wait_for_service('/ur_hardware_interface/set_io', timeout=2.0)
    except rospy.ROSException:
        rospy.logerr("Servicio /ur_hardware_interface/set_io no disponible. No se puede controlar el gripper.")
        return False
    
    try:
        set_io = rospy.ServiceProxy('/ur_hardware_interface/set_io', SetIO)
        # Para 'SetIO', el 'fun' es 1 para DO, 'pin' es el número de pin, 'state' es 0.0 o 1.0
        resp = set_io(fun=1, pin=pin, state=float(state)) # Asegurarse que state es float
        if resp.success:
            rospy.loginfo(f"GRIPPER: Pin digital {pin} establecido en {state}")
        else:
            rospy.logwarn(f"GRIPPER: Fallo al establecer pin {pin} en {state}")
        return resp.success
    except rospy.ServiceException as e:
        rospy.logerr(f"GRIPPER: Error en el servicio de IO: {e}")
        return False

def calculate_joint_angles_from_3d_points(shoulder_3d, elbow_3d, wrist_3d):
    """
    Calcula los ángulos articulares para shoulder_lift, elbow y wrist_1
    usando la posición 3D del hombro, codo y muñeca.
    Estos puntos ya deben estar en el sistema de coordenadas del robot.
    """
    v_upper_arm = elbow_3d - shoulder_3d
    v_forearm = wrist_3d - elbow_3d

    shoulder_pan_angle = math.atan2(v_upper_arm[1], v_upper_arm[0])
    len_v_upper_arm_xy = math.sqrt(v_upper_arm[0]**2 + v_upper_arm[1]**2)
    if len_v_upper_arm_xy < 1e-6 : # Avoid division by zero if arm is pointing straight up/down
        shoulder_lift_angle = math.pi/2 if -v_upper_arm[2] > 0 else -math.pi/2
    else:
        shoulder_lift_angle = math.atan2(-v_upper_arm[2], len_v_upper_arm_xy)

    norm_v_upper_arm = np.linalg.norm(v_upper_arm)
    norm_v_forearm = np.linalg.norm(v_forearm)
    if norm_v_upper_arm < 1e-6 or norm_v_forearm < 1e-6:
        elbow_angle = 0.0
    else:
        dot_product = np.dot(-v_upper_arm, v_forearm)
        elbow_angle_rad = math.acos(np.clip(dot_product / (norm_v_upper_arm * norm_v_forearm), -1.0, 1.0))
        elbow_angle = elbow_angle_rad - math.pi

    # Simplificaciones para los ángulos de la muñeca.
    # wrist_1: rotación del antebrazo. Se puede mantener en 0 o intentar derivar.
    # wrist_2: flexión/extensión de la muñeca. Para apuntar hacia abajo: -pi/2
    # wrist_3: rotación de la herramienta.
    wrist_1_angle = 0.0  # Mantener neutro o calcular más sofisticadamente
    wrist_2_angle = -math.pi / 2 # Ejemplo: apuntar el gripper hacia abajo
    wrist_3_angle = 0.0 # Ejemplo: sin rotación de herramienta adicional

    joint_angles = [
        shoulder_pan_angle,
        shoulder_lift_angle,
        elbow_angle,
        wrist_1_angle,
        wrist_2_angle,
        wrist_3_angle
    ]
    return joint_angles
# Almacenar el ángulo de la muñeca del robot en la posición de agarre
# Esto podría ser un valor fijo o el calculado cuando la mano humana agarra.
WRIST_1_ANGLE_AT_GRASP_ROBOT = 0.0 # Ángulo de referencia para wrist_1 del robot

def calculate_joint_angles_hybrid(shoulder_3d, elbow_3d, wrist_3d,
                                   human_wrist_rotation_delta=0.0,
                                   target_wrist_2_angle=-math.pi/2, # Ejemplo: apuntar hacia abajo
                                   target_wrist_3_angle=0.0):
    """
    Calcula los primeros 3 joints basados en la pose.
    Calcula wrist_1 basado en la rotación de la muñeca humana.
    Fija wrist_2 y wrist_3.
    """
    # --- Cálculos para shoulder_pan, shoulder_lift, elbow (como antes) ---
    v_upper_arm = elbow_3d - shoulder_3d
    v_forearm = wrist_3d - elbow_3d

    shoulder_pan_angle = math.atan2(v_upper_arm[1], v_upper_arm[0])
    len_v_upper_arm_xy = math.sqrt(v_upper_arm[0]**2 + v_upper_arm[1]**2)
    if len_v_upper_arm_xy < 1e-6 :
        shoulder_lift_angle = math.pi/2 if -v_upper_arm[2] > 0 else -math.pi/2
    else:
        shoulder_lift_angle = math.atan2(-v_upper_arm[2], len_v_upper_arm_xy)

    norm_v_upper_arm = np.linalg.norm(v_upper_arm)
    norm_v_forearm = np.linalg.norm(v_forearm)
    if norm_v_upper_arm < 1e-6 or norm_v_forearm < 1e-6:
        elbow_angle = 0.0
    else:
        dot_product = np.dot(-v_upper_arm, v_forearm)
        elbow_angle_rad = math.acos(np.clip(dot_product / (norm_v_upper_arm * norm_v_forearm), -1.0, 1.0))
        elbow_angle = elbow_angle_rad - math.pi # 0 es recto
    # --- Fin cálculos primeros 3 joints ---

    # --- wrist_1_angle ---
    # Asumimos que human_wrist_rotation_delta es el cambio de ángulo desde el agarre
    # Podría necesitar un factor de escala si el rango humano y robot no coinciden.
    # Y un posible cambio de signo dependiendo de las convenciones.
    wrist_1_angle_robot = WRIST_1_ANGLE_AT_GRASP_ROBOT + math.radians(human_wrist_rotation_delta)
    # Asegúrate de que el ángulo está en el rango del robot, ej. [-2*pi, 2*pi] o [-pi, pi]
    # wrist_1_angle_robot = np.clip(wrist_1_angle_robot, -math.pi, math.pi) # Ejemplo de clip

    # --- wrist_2 y wrist_3 ---
    wrist_2_angle_robot = target_wrist_2_angle
    wrist_3_angle_robot = target_wrist_3_angle

    joint_angles = [
        shoulder_pan_angle,
        shoulder_lift_angle,
        elbow_angle,
        wrist_1_angle_robot,
        wrist_2_angle_robot,
        wrist_3_angle_robot
    ]
    return joint_angles

def go_to_joint_state(move_group, joints, wait=True):
    """Mueve el robot a un estado articular específico."""
    move_group.set_joint_value_target(joints)
    rospy.loginfo(f"Planificando a joints: {[f'{q:.3f}' for q in joints]}")
    plan_success, plan, _, _ = move_group.plan()

    if plan_success:
        rospy.loginfo(f"Ejecutando plan articular...")
        move_group.execute(plan, wait=wait)
        if wait:
            rospy.loginfo("Movimiento articular completo.")
        return True
    else:
        rospy.logerr("No se encontró plan articular válido.")
        return False

def find_latest_segment_folder(base_dir_pattern):
    """Encuentra la carpeta de segmentos más reciente."""
    list_of_folders = glob.glob(base_dir_pattern)
    if not list_of_folders:
        return None
    latest_folder = max(list_of_folders, key=os.path.getctime)
    return latest_folder

def load_trajectory_from_csv(folder_path, phase_name_pattern):
    """Carga la primera trayectoria que coincida con el patrón de phase_name."""
    search_pattern = os.path.join(folder_path, f"segment_*_{phase_name_pattern}_*.csv")
    matching_files = glob.glob(search_pattern)

    if not matching_files:
        rospy.logwarn(f"No se encontraron archivos CSV para el patrón de fase '{phase_name_pattern}' en '{folder_path}'")
        return None

    trajectory_file = sorted(matching_files)[0] 
    rospy.loginfo(f"Cargando trayectoria desde: {trajectory_file}")
    try:
        df = pd.read_csv(trajectory_file)
        expected_cols_base = ["right_shoulder", "right_elbow", "right_hand"]
        expected_cols_full = []
        for base in expected_cols_base:
            expected_cols_full.extend([f"{base}_x", f"{base}_y", f"{base}_z"])
        
        if not all(col in df.columns for col in expected_cols_full):
            rospy.logerr(f"CSV {trajectory_file} no tiene columnas esperadas. Encontradas: {df.columns.tolist()}")
            return None
        return df
    except Exception as e:
        rospy.logerr(f"Error al cargar CSV {trajectory_file}: {e}")
        return None


# ===== MAIN =====
if __name__ == '__main__':
    try:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('ur3e_full_trajectory_player', anonymous=True)

        group_name = "manipulator"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        move_group.set_planning_time(10.0)
        move_group.allow_replanning(True)
        move_group.set_goal_joint_tolerance(0.01)
        move_group.set_num_planning_attempts(5)

        rospy.loginfo("Robot inicializado. Yendo a HOME.")
        if not go_to_joint_state(move_group, HOME_JOINTS):
            rospy.logerr("Fallo al ir a HOME. Abortando.")
            sys.exit()
        time.sleep(1)

        latest_segments_folder = find_latest_segment_folder(ACTION_SEGMENTS_BASE_DIR)
        if not latest_segments_folder:
            rospy.logerr(f"No se encontraron carpetas de segmentos. Abortando.")
            sys.exit()
        rospy.loginfo(f"Usando carpeta de segmentos: {latest_segments_folder}")

   # Asumimos que trajectory_df ahora tiene una columna 'right_hand_rotation_angle_value'
        # Y que esta columna contiene el *delta de ángulo* desde el agarre, en grados.
        # O, si guarda el ángulo absoluto, necesitarías el 'angle_at_grasp' del humano también.
        # Supongamos que guardaste el delta directamente: human_angle_delta_degrees

        latest_segments_folder = find_latest_segment_folder(ACTION_SEGMENTS_BASE_DIR) # find_latest_segment_folder definido antes
        if not latest_segments_folder:
            rospy.logerr(f"No se encontraron carpetas de segmentos. Abortando.")
            sys.exit()
        rospy.loginfo(f"Usando carpeta de segmentos: {latest_segments_folder}")


        for phase_idx, target_phase in enumerate(TASK_PHASE_SEQUENCE): # TASK_PHASE_SEQUENCE definido antes
            if rospy.is_shutdown(): break
            rospy.loginfo(f"\n--- Iniciando Fase: {target_phase} ---")

            phase_pattern_to_load = target_phase
            if target_phase == "Rotating-Separated":
                phase_pattern_to_load = "Rotating-*"

            trajectory_df = load_trajectory_from_csv(latest_segments_folder, phase_pattern_to_load) # load_trajectory_from_csv definido antes

            if trajectory_df is not None:
                rospy.loginfo(f"Procesando {len(trajectory_df)} puntos para '{target_phase}'...")

                # Lógica de Gripper
                if target_phase == "Initial-Idle":
                    control_gripper(GRIPPER_PIN, GRIPPER_STATE_OPEN) # control_gripper definido antes
                    time.sleep(0.5)
                elif target_phase == "Rotating-Separated":
                    rospy.loginfo("Acción: CERRAR GRIPPER (simulado)")
                    control_gripper(GRIPPER_PIN, GRIPPER_STATE_CLOSE)
                    time.sleep(1)

                for index, row in trajectory_df.iterrows():
                    if rospy.is_shutdown(): break

                    shoulder_cam = np.array([row['right_shoulder_x'], row['right_shoulder_y'], row['right_shoulder_z']])
                    elbow_cam = np.array([row['right_elbow_x'], row['right_elbow_y'], row['right_elbow_z']])
                    wrist_cam = np.array([row['right_hand_x'], row['right_hand_y'], row['right_hand_z']])

                    # Obtener el delta de rotación de la muñeca humana (en grados)
                    # Asegúrate de que esta columna existe en tu CSV
                    human_angle_delta_degrees = 0.0
                    if 'right_hand_rotation_angle_delta_value' in row: # Asumiendo que guardas el delta
                        human_angle_delta_degrees = row['right_hand_rotation_angle_delta_value']
                    elif 'right_hand_rotation_angle_value' in row and 'human_angle_at_grasp' in trajectory_df.attrs: # Si guardas absoluto y el angulo de agarre
                        human_angle_delta_degrees = row['right_hand_rotation_angle_value'] - trajectory_df.attrs['human_angle_at_grasp']


                    shoulder_robot = shoulder_cam + GLOBAL_OFFSET
                    elbow_robot = elbow_cam + GLOBAL_OFFSET
                    wrist_robot = wrist_cam + GLOBAL_OFFSET

                    target_joint_angles = calculate_joint_angles_hybrid(
                        shoulder_robot, elbow_robot, wrist_robot,
                        human_wrist_rotation_delta=human_angle_delta_degrees
                    )

                    if not go_to_joint_state(move_group, target_joint_angles, wait=True):
                        rospy.logwarn(f"Fallo al planificar/ejecutar para punto {index} de fase '{target_phase}'.")
                        # break 

                rospy.loginfo(f"Fase '{target_phase}' completada.")

                if target_phase == "Rotating-Separated":
                    rospy.loginfo("Acción: ABRIR GRIPPER (simulado)")
                    control_gripper(GRIPPER_PIN, GRIPPER_STATE_OPEN)
                    time.sleep(1)
            else:
                rospy.logwarn(f"No se pudo cargar la trayectoria para la fase '{target_phase}'. Saltando.")

            if not rospy.is_shutdown() and phase_idx < len(TASK_PHASE_SEQUENCE) - 1:
                rospy.loginfo("Pausa antes de la siguiente fase..."); time.sleep(0.5)

        rospy.loginfo("\n--- Secuencia de Tarea Completa Finalizada ---")
        rospy.loginfo("Volviendo a HOME.")
        go_to_joint_state(move_group, HOME_JOINTS)


    except rospy.ROSInterruptException:
        rospy.loginfo("Programa interrumpido por ROS.")
    except Exception as e:
        rospy.logerr(f"Excepción no controlada en main: {e}")
        import traceback
        traceback.print_exc()
    finally:
        rospy.loginfo("Finalizando MoveIt Commander.")
        moveit_commander.roscpp_shutdown()
        rospy.loginfo("Programa finalizado.")