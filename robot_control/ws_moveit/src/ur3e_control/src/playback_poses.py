#!/usr/bin/env python3

import rospy
import moveit_commander
import sys
import time
from geometry_msgs.msg import Pose

pose1 = Pose()
pose1.position.x = -3.2631711406697054e-05
pose1.position.y = 0.22314999953670325
pose1.position.z = 0.693949997707084
pose1.orientation.x = -0.7071067758100374
pose1.orientation.y = -0.00010167173887972231
pose1.orientation.z = -6.946081965704481e-05
pose1.orientation.w = 0.7071067758419413

pose2 = Pose()

pose2.position.x = -0.08261064963600667
pose2.position.y = 0.22196363306244543
pose2.position.z = 0.6873480494524089
pose2.orientation.x = -0.33154488497129553
pose2.orientation.y = -0.6928248590132944
pose2.orientation.z = -0.6023522020025535
pose2.orientation.w = 0.2173557653370481

def go_to_pose_goal(pose_target):
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('ur3e_go_to_pose_goal', anonymous=True)

    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    move_group.set_planning_time(10.0)
    move_group.set_pose_reference_frame("base_link")  # ← Important addition



    rospy.loginfo("Moviendo a una pose cartesiana conocida...")
    rospy.loginfo(f"Pose actual: {move_group.get_current_pose().pose}")
    pose_target.orientation = move_group.get_current_pose().pose.orientation
    rospy.loginfo(f"Objetivo pose: {pose_target}")

    move_group.set_start_state_to_current_state()
    move_group.set_pose_target(pose_target)
    ik_result = move_group.set_joint_value_target(pose_target, True)
    if not ik_result:
        rospy.logerr("Inverse kinematics could not find a solution.")
        return
    plan_success, plan, _, _ = move_group.plan()

    if plan_success:
        rospy.loginfo("Plan cartesiano encontrado. Ejecutando...")
        move_group.execute(plan, wait=True)
        rospy.loginfo("Movimiento a posición cartesiana completado.")
        rospy.loginfo(f"Nueva pose cartesiana: {move_group.get_current_pose().pose}")
    else:
        rospy.logerr("No se encontró un plan válido para la pose objetivo.")
        rospy.logwarn("Verifica si la pose es alcanzable desde el estado actual o prueba con otra orientación.")

    move_group.stop()
    move_group.clear_pose_targets()

if __name__ == '__main__':
    try:
        go_to_pose_goal(pose2)
        time.sleep(2)
        rospy.loginfo("move1 completed.")
        go_to_pose_goal(pose1)
        rospy.loginfo("move2 completed.")
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Excepción no controlada: {e}")
    finally:
        moveit_commander.roscpp_shutdown()
