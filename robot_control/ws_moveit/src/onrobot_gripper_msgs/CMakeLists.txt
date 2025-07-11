cmake_minimum_required(VERSION 3.0.2)
project(onrobot_gripper_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geometry_msgs
  roscpp
  rospy
  std_msgs
  trajectory_msgs
  message_generation
)

add_message_files(
   DIRECTORY msg
   FILES
   JointControllerState.msg
   OnrobotGripperCommand.msg
)

add_action_files(
   DIRECTORY action
   FILES
   OnrobotGripperCommand.action
)
## Generate added messages and services with any dependencies listed here
generate_messages( DEPENDENCIES trajectory_msgs std_msgs geometry_msgs
   actionlib_msgs)

catkin_package(
    CATKIN_DEPENDS actionlib_msgs trajectory_msgs geometry_msgs message_runtime
)
