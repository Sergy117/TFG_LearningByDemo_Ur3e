#!/usr/bin/env python3

import rospy
import moveit_commander
import sys
import math # Para usar math.pi
import time
import keyboard

from ur_msgs.srv import SetIO


pos1 = [0.29374200105667114, -2.5416180096068324, -1.2833071947097778, 0.4725138384052734, -0.33912450471986944, -0.006049935017720998]
home = [0.004553189035505056, -1.5730906925597132, 0.011014286671773732, -1.5756780109801234, 0.0015274136094376445, -0.006227795277730763]

release = [-2.0775626341449183, -1.9422828159727992, -1.6426301002502441, -1.1176169377616425, 1.4957866668701172, 2.1914095878601074]
grab = [-0.16444856325258428, -2.1855503521361292, -1.18836510181427, -1.4002882105163117, 1.5105690956115723, 0.9914141297340393]
def control_gripper(pin, state):
    rospy.wait_for_service('/ur_hardware_interface/set_io')
    try:
        set_io = rospy.ServiceProxy('/ur_hardware_interface/set_io', SetIO)
        resp = set_io(1, pin, state)  # 1: digital_out, pin: 16/17, state: 0.0 or 1.0
        if resp.success:
            rospy.loginfo(f"Salida digital {pin} establecida en {state}")
        else:
            rospy.logwarn(f"Falló el intento de establecer pin {pin} en {state}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Error llamando al servicio de IO: {e}")

def go_to_joint_state(joints):
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('ur3e_go_to_joint_state', anonymous=True)

    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    move_group.set_planning_time(10.0)

    rospy.loginfo("Moviendo a una posición articular conocida...")

    # [shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]
    # Una posición "vertical" o "home" común:
    #target_joint_positions = [0.0, -math.pi/2, -math.pi/2, -math.pi/2, math.pi/2, 0.0]
    # target_joint_positions = [0.0, -1.8, -1.5, -1.3, math.pi/2, 0.0]
    target_joint_positions = joints

    rospy.loginfo(f"Valores articulares actuales: {move_group.get_current_joint_values()}")
    rospy.loginfo(f"Objetivo articular: {target_joint_positions}")

    move_group.set_joint_value_target(target_joint_positions)

    plan_success, plan, _, _ = move_group.plan()

    if plan_success:
        rospy.loginfo("Plan articular encontrado. Ejecutando...")
        move_group.execute(plan, wait=True)
        rospy.loginfo("Movimiento a posición articular completado.")
        rospy.loginfo(f"Nuevos valores articulares: {move_group.get_current_joint_values()}")
        rospy.loginfo(f"Nueva pose cartesiana: {move_group.get_current_pose().pose}")
    else:
        rospy.logerr("No se encontró plan para la posición articular objetivo.")

    move_group.stop()
    # No es necesario clear_pose_targets() si solo usaste set_joint_value_target()
    # move_group.clear_joint_value_targets() # No existe esta función directamente

if __name__ == '__main__':

    try:
        go_to_joint_state(home)
        time.sleep(1)

        # Ir a la posición de agarre
        go_to_joint_state(grab)
        time.sleep(1)

        # Cerrar la pinza (activar pin 17)
        control_gripper(16, 0.0)  # Asegúrate de que "abrir" esté apagado
        control_gripper(17, 1.0)
        time.sleep(1)

        go_to_joint_state(home)
        time.sleep(1)

        # Ir a la posición de liberación
        go_to_joint_state(release)
        time.sleep(1)

        # Abrir la pinza (activar pin 16)
        control_gripper(17, 0.0)  # Asegúrate de que "cerrar" esté apagado
        control_gripper(16, 1.0)
        time.sleep(1)

        go_to_joint_state(home)
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Excepción no controlada: {e}")
    finally:
        moveit_commander.roscpp_shutdown()
