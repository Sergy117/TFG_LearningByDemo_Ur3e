#!/usr/bin/env python3

import rospy
import moveit_commander
import sys
import time
import math
import numpy as np
import os
import pickle
import pydmps.dmp_discrete
from ur_msgs.srv import SetIO

# --- Configuración ---
# Directorio donde guardaste los DMPs aprendidos
DMP_DIR = "/home/robotica/ws_moveit/src/ur3e_control/src/Dmps_fin"
HOME_JOINTS = [0.0, -math.pi/2, 0.0, -math.pi/2, 0.0, 0.0] # Home estándar

# Secuencia de fases para ejecutar la tarea completa
TASK_PHASE_SEQUENCE = [
    "initial_idle",
    "grasping_to_rotating",
    "rotating_separated",
]
UR3E_JOINT_LIMITS = {
    'shoulder_pan_joint': (-2*math.pi, 2*math.pi),
    'shoulder_lift_joint': (-2*math.pi, 2*math.pi),
    'elbow_joint': (-math.pi, math.pi),
    'wrist_1_joint': (-2*math.pi, 2*math.pi),
    'wrist_2_joint': (-2*math.pi, 2*math.pi),
    'wrist_3_joint': (-2*math.pi, 2*math.pi)
}
# Configuración del Gripper
GRIPPER_PIN = 0
GRIPPER_STATE_OPEN = 0.0
GRIPPER_STATE_CLOSE = 1.0


# --- Funciones (control_gripper y go_to_joint_state sin cambios) ---
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
def clip_joint_angles(joint_angles, joint_names, limits):
    """
    Recorta los ángulos de las articulaciones para que estén dentro de los límites definidos.
    """
    clipped_angles = []
    for i, joint_name in enumerate(joint_names):
        # El orden de los nombres debe coincidir con el de los ángulos
        if joint_name in limits:
            min_lim, max_lim = limits[joint_name]
            clipped_angle = np.clip(joint_angles[i], min_lim, max_lim)
            clipped_angles.append(clipped_angle)
        else:
            rospy.logwarn(f"El joint '{joint_name}' no tiene límites definidos en el script. Se usará el valor original.")
            clipped_angles.append(joint_angles[i])
    return clipped_angles
def go_to_joint_state(move_group, joints, wait=True):
    move_group.set_joint_value_target(joints)
    plan_success, plan, _, _ = move_group.plan()
    if plan_success:
        move_group.execute(plan, wait=wait)
        return True
    else:
        rospy.logerr("No se encontró plan articular válido.")
        return False


def load_all_dmps(dmp_dir, phase_sequence):
    """Carga todos los archivos DMP .pkl necesarios para la secuencia de la tarea."""
    loaded_dmps = {}
    print("Cargando modelos DMP aprendidos...")
    for phase in phase_sequence:
        dmp_path = os.path.join(dmp_dir, f"dmp_{phase}.pkl")
        if not os.path.exists(dmp_path):
            rospy.logerr(f"ERROR: No se encontró el archivo DMP para la fase '{phase}' en {dmp_path}. Abortando.")
            return None
        try:
            with open(dmp_path, 'rb') as f:
                loaded_dmps[phase] = pickle.load(f)
            rospy.loginfo(f"  -> DMP para '{phase}' cargado exitosamente.")
        except Exception as e:
            rospy.logerr(f"Error al cargar el archivo DMP {dmp_path}: {e}")
            return None
    return loaded_dmps

# ===== MAIN =====
if __name__ == '__main__':
    try:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('ur3e_dmp_player', anonymous=True)
        group_name = "manipulator"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        move_group.set_planning_time(5.0)
        move_group.set_max_velocity_scaling_factor(0.5) # Empezar con una velocidad reducida por seguridad
        move_group.set_max_acceleration_scaling_factor(0.5)
        active_joint_names = move_group.get_active_joints()

        # 1. Cargar todos los DMPs necesarios
        all_dmps = load_all_dmps(DMP_DIR, TASK_PHASE_SEQUENCE)
        if all_dmps is None:
            sys.exit()

        rospy.loginfo("Robot inicializado y DMPs cargados. Yendo a HOME.")
        go_to_joint_state(move_group, HOME_JOINTS)
        time.sleep(1)

        # 2. Bucle para ejecutar la secuencia de DMPs
        current_robot_joints = np.array(move_group.get_current_joint_values())

        for phase_idx, phase_name in enumerate(TASK_PHASE_SEQUENCE):
            if rospy.is_shutdown(): break
            rospy.loginfo(f"\n--- Ejecutando Fase DMP: {phase_name} ---")

            # Cargar los datos del DMP para esta fase
            dmp_data = all_dmps[phase_name]
            dmp = dmp_data['dmp_model']
            dmp_goal = dmp_data['goal_pose'] # El punto final de la trayectoria promedio aprendida

            # Establecer el punto de inicio y fin para el DMP
            dmp.y0 = current_robot_joints  # El inicio es donde el robot está ahora
            dmp.goal = dmp_goal            # El objetivo es el final de la demo promedio
            
            # Generar la trayectoria completa del DMP
            # El DMP genera una trayectoria de (n_timesteps, n_joints)
            dmp_trajectory, _, _ = dmp.rollout()
            rospy.loginfo(f"Generada trayectoria de {len(dmp_trajectory)} puntos para la fase '{phase_name}'.")

            # Lógica de Gripper
            if phase_name == "initial_idle":
                rospy.loginfo("Acción: Gripper Abierto initial idle")
                control_gripper(16, 1); time.sleep(0.5)
                control_gripper(17, 0); time.sleep(0.5)
            elif phase_name == "grasping_to_rotating":
                rospy.loginfo("Acción: Gripper Cerrado grasping to rotating")
                control_gripper(16, 0); time.sleep(0.5)
                control_gripper(17, 1); time.sleep(0.5)
            elif phase_name == "rotating_separated":
                rospy.loginfo("Acción: Gripper Cerrado rotating to separated")
                #control_gripper(17, 0); time.sleep(0.5)
                #control_gripper(16, 1); time.sleep(0.5)

            # 3. Ejecutar la trayectoria generada punto por punto
            if phase_name != "grasping_to_rotating":
                for joint_target in dmp_trajectory:
                    if rospy.is_shutdown(): break
                    
                    # Para un movimiento más suave, MoveIt puede planificar a través de una secuencia de waypoints.
                    # Pero para empezar, ir punto a punto es más fácil de depurar.
                    clipped_joint_target = clip_joint_angles(joint_target, active_joint_names, UR3E_JOINT_LIMITS)
                    
                    # Enviar los ángulos YA RECORTADOS y seguros al robot
                    if not go_to_joint_state(move_group, clipped_joint_target, wait=True):
                        rospy.logwarn(f"Fallo al ejecutar un punto de la trayectoria DMP. Abortando fase.")
                        break
            else:
                 # --- LÓGICA ESPECIAL PARA LA FASE DE ROTACIÓN ---
                rospy.loginfo("Acción: Cerrar Gripper para agarrar.")
                control_gripper(16, 0); control_gripper(17, 1); time.sleep(1)

                rospy.loginfo("MODO ROTACIÓN PURA: Manteniendo brazo y orientación de muñeca fijos, girando solo la herramienta (wrist_3).")
                
                # Guardar la posición actual de los primeros 5 joints (todo excepto el de giro final)
                fixed_base_joints = move_group.get_current_joint_values()[:5]
                
                for joint_target_from_dmp in dmp_trajectory:
                    if rospy.is_shutdown(): break
                    
                    # Extraer el valor de rotación que el DMP aprendió (que estaba destinado a wrist_1)
                    rotation_value = joint_target_from_dmp[3] 
                    
                    # Construir el objetivo final:
                    # - Usar los 5 joints FIJOS del brazo y la muñeca
                    # - Usar el valor de rotación para el SEXTO joint (wrist_3)
                    final_joint_target = list(fixed_base_joints) + [rotation_value]
                    
                    clipped_target = clip_joint_angles(final_joint_target, active_joint_names, UR3E_JOINT_LIMITS)
                    if not go_to_joint_state(move_group, clipped_target, wait=True):
                        rospy.logwarn("Fallo al ejecutar la rotación pura de muñeca. Abortando."); break
            # --- <<< FIN DEL BLOQUE MODIFICADO >>> ---

            rospy.loginfo(f"Fase '{phase_name}' completada.")
            # Actualizar el estado actual del robot para la siguiente fase
            current_robot_joints = np.array(move_group.get_current_joint_values())


        rospy.loginfo("\n--- TAREA COMPLETA EJECUTADA CON DMPS ---")
        rospy.loginfo("Volviendo a HOME.")
        control_gripper(17, 0); time.sleep(0.5)
        control_gripper(16, 1); time.sleep(0.5)
        go_to_joint_state(move_group, HOME_JOINTS)

    except rospy.ROSInterruptException: rospy.loginfo("Programa interrumpido.")
    except Exception as e: rospy.logerr(f"Excepción no controlada: {e}"); import traceback; traceback.print_exc()
    finally: control_gripper(17, 0);control_gripper(16, 0); time.sleep(0.5);rospy.loginfo("Finalizando."); moveit_commander.roscpp_shutdown()