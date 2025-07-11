#!/usr/bin/env python3
import rospy
import moveit_commander
import geometry_msgs.msg
import json
import os
import time
saved_poses = []


if __name__ == '__main__':
    rospy.init_node('save_pose_node')
    moveit_commander.roscpp_initialize([])
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    move_group = moveit_commander.MoveGroupCommander("manipulator")
    while True:
        #pose = move_group.get_current_joint_values()
        pose = move_group.get_current_pose().pose
        rospy.loginfo(f"Jonts: {pose}")
        time.sleep(5)
        input("Presiona ENTER")
