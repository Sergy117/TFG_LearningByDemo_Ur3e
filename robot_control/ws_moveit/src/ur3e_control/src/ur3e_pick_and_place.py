#!/usr/bin/env python3

import rospy
import moveit_commander
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from ur_msgs.srv import SetIO

import tf.transformations as tf

def set_tool_digital_output(pin, state):
    try:
        rospy.wait_for_service('/ur_hardware_interface/set_io', timeout=5.0)
        set_io = rospy.ServiceProxy('/ur_hardware_interface/set_io', SetIO)
        result = set_io(fun=1, pin=pin, state=float(state))
        if result.success:
            rospy.loginfo(f"Salida digital del pin {pin} cambiada a {state}")
        else:
            rospy.logwarn("Fallo al cambiar salida digital")
    except rospy.ServiceException as e:
        rospy.logerr(f"Servicio set_io no disponible: {e}")

def main():
    rospy.init_node('ur3e_pick_and_place', anonymous=True)
    moveit_commander.roscpp_initialize([])

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("manipulator")

    # Esperar a MoveIt
    rospy.sleep(2)

    # Paso 1: Cerrar la pinza
    set_tool_digital_output(pin=0, state=1)  # 1 = cerrar

    # Paso 2: Definir pose segura
    pose_target = Pose()
    pose_target.position.x = -0.28
    pose_target.position.y = -0.33
    pose_target.position.z = 0.25

    q = tf.quaternion_from_euler(0.517, -1.951, 1.193)
    pose_target.orientation.x = q[0]
    pose_target.orientation.y = q[1]
    pose_target.orientation.z = q[2]
    pose_target.orientation.w = q[3]

    group.set_pose_target(pose_target)

    # Paso 3: Planificar y mover
    plan = group.plan()

    if plan and plan[0]:
        group.execute(plan[1], wait=True)
        rospy.loginfo("Movimiento completado.")
    else:
        rospy.logwarn("Falló la planificación. No se ejecuta movimiento.")

    # Paso 4: Abrir la pinza
    rospy.sleep(1)
    set_tool_digital_output(pin=0, state=0)  # 0 = abrir

    group.clear_pose_targets()

    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
