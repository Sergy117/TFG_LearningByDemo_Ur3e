cmake_minimum_required(VERSION 3.0.2)
project(onrobot_gripper_action)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS roscpp actionlib_msgs actionlib onrobot_gripper_msgs sensor_msgs tm_msgs)

find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

catkin_package()

add_executable(onrobot_gripper_action src/onrobot_gripper_action.cpp)
target_link_libraries(onrobot_gripper_action ${Boost_LIBRARIES} ${catkin_LIBRARIES})

install(TARGETS onrobot_gripper_action
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
   


add_executable(joint_states_updater src/joint_states_updater.cpp)
target_link_libraries(joint_states_updater ${Boost_LIBRARIES} ${catkin_LIBRARIES})


