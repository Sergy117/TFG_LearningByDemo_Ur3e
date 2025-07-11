# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "onrobot_gripper_msgs: 9 messages, 0 services")

set(MSG_I_FLAGS "-Ionrobot_gripper_msgs:/home/robotica/ws_moveit/src/onrobot_gripper_msgs/msg;-Ionrobot_gripper_msgs:/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg;-Itrajectory_msgs:/opt/ros/noetic/share/trajectory_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/noetic/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(onrobot_gripper_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/robotica/ws_moveit/src/onrobot_gripper_msgs/msg/JointControllerState.msg" NAME_WE)
add_custom_target(_onrobot_gripper_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "onrobot_gripper_msgs" "/home/robotica/ws_moveit/src/onrobot_gripper_msgs/msg/JointControllerState.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/robotica/ws_moveit/src/onrobot_gripper_msgs/msg/OnrobotGripperCommand.msg" NAME_WE)
add_custom_target(_onrobot_gripper_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "onrobot_gripper_msgs" "/home/robotica/ws_moveit/src/onrobot_gripper_msgs/msg/OnrobotGripperCommand.msg" ""
)

get_filename_component(_filename "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandAction.msg" NAME_WE)
add_custom_target(_onrobot_gripper_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "onrobot_gripper_msgs" "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandAction.msg" "onrobot_gripper_msgs/OnrobotGripperCommandResult:onrobot_gripper_msgs/OnrobotGripperCommandFeedback:onrobot_gripper_msgs/OnrobotGripperCommand:onrobot_gripper_msgs/OnrobotGripperCommandActionGoal:onrobot_gripper_msgs/OnrobotGripperCommandGoal:actionlib_msgs/GoalStatus:onrobot_gripper_msgs/OnrobotGripperCommandActionFeedback:actionlib_msgs/GoalID:onrobot_gripper_msgs/OnrobotGripperCommandActionResult:std_msgs/Header"
)

get_filename_component(_filename "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandActionGoal.msg" NAME_WE)
add_custom_target(_onrobot_gripper_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "onrobot_gripper_msgs" "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandActionGoal.msg" "onrobot_gripper_msgs/OnrobotGripperCommandGoal:std_msgs/Header:onrobot_gripper_msgs/OnrobotGripperCommand:actionlib_msgs/GoalID"
)

get_filename_component(_filename "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandActionResult.msg" NAME_WE)
add_custom_target(_onrobot_gripper_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "onrobot_gripper_msgs" "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandActionResult.msg" "onrobot_gripper_msgs/OnrobotGripperCommandResult:std_msgs/Header:actionlib_msgs/GoalStatus:actionlib_msgs/GoalID"
)

get_filename_component(_filename "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandActionFeedback.msg" NAME_WE)
add_custom_target(_onrobot_gripper_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "onrobot_gripper_msgs" "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandActionFeedback.msg" "onrobot_gripper_msgs/OnrobotGripperCommandFeedback:std_msgs/Header:actionlib_msgs/GoalStatus:actionlib_msgs/GoalID"
)

get_filename_component(_filename "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandGoal.msg" NAME_WE)
add_custom_target(_onrobot_gripper_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "onrobot_gripper_msgs" "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandGoal.msg" "onrobot_gripper_msgs/OnrobotGripperCommand"
)

get_filename_component(_filename "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandResult.msg" NAME_WE)
add_custom_target(_onrobot_gripper_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "onrobot_gripper_msgs" "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandResult.msg" ""
)

get_filename_component(_filename "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandFeedback.msg" NAME_WE)
add_custom_target(_onrobot_gripper_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "onrobot_gripper_msgs" "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandFeedback.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(onrobot_gripper_msgs
  "/home/robotica/ws_moveit/src/onrobot_gripper_msgs/msg/JointControllerState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/onrobot_gripper_msgs
)
_generate_msg_cpp(onrobot_gripper_msgs
  "/home/robotica/ws_moveit/src/onrobot_gripper_msgs/msg/OnrobotGripperCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/onrobot_gripper_msgs
)
_generate_msg_cpp(onrobot_gripper_msgs
  "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandAction.msg"
  "${MSG_I_FLAGS}"
  "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandResult.msg;/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandFeedback.msg;/home/robotica/ws_moveit/src/onrobot_gripper_msgs/msg/OnrobotGripperCommand.msg;/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandActionGoal.msg;/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandGoal.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandActionFeedback.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandActionResult.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/onrobot_gripper_msgs
)
_generate_msg_cpp(onrobot_gripper_msgs
  "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandGoal.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/robotica/ws_moveit/src/onrobot_gripper_msgs/msg/OnrobotGripperCommand.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/onrobot_gripper_msgs
)
_generate_msg_cpp(onrobot_gripper_msgs
  "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandActionResult.msg"
  "${MSG_I_FLAGS}"
  "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandResult.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/onrobot_gripper_msgs
)
_generate_msg_cpp(onrobot_gripper_msgs
  "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/onrobot_gripper_msgs
)
_generate_msg_cpp(onrobot_gripper_msgs
  "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/robotica/ws_moveit/src/onrobot_gripper_msgs/msg/OnrobotGripperCommand.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/onrobot_gripper_msgs
)
_generate_msg_cpp(onrobot_gripper_msgs
  "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/onrobot_gripper_msgs
)
_generate_msg_cpp(onrobot_gripper_msgs
  "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/onrobot_gripper_msgs
)

### Generating Services

### Generating Module File
_generate_module_cpp(onrobot_gripper_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/onrobot_gripper_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(onrobot_gripper_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(onrobot_gripper_msgs_generate_messages onrobot_gripper_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/robotica/ws_moveit/src/onrobot_gripper_msgs/msg/JointControllerState.msg" NAME_WE)
add_dependencies(onrobot_gripper_msgs_generate_messages_cpp _onrobot_gripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotica/ws_moveit/src/onrobot_gripper_msgs/msg/OnrobotGripperCommand.msg" NAME_WE)
add_dependencies(onrobot_gripper_msgs_generate_messages_cpp _onrobot_gripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandAction.msg" NAME_WE)
add_dependencies(onrobot_gripper_msgs_generate_messages_cpp _onrobot_gripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandActionGoal.msg" NAME_WE)
add_dependencies(onrobot_gripper_msgs_generate_messages_cpp _onrobot_gripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandActionResult.msg" NAME_WE)
add_dependencies(onrobot_gripper_msgs_generate_messages_cpp _onrobot_gripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandActionFeedback.msg" NAME_WE)
add_dependencies(onrobot_gripper_msgs_generate_messages_cpp _onrobot_gripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandGoal.msg" NAME_WE)
add_dependencies(onrobot_gripper_msgs_generate_messages_cpp _onrobot_gripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandResult.msg" NAME_WE)
add_dependencies(onrobot_gripper_msgs_generate_messages_cpp _onrobot_gripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandFeedback.msg" NAME_WE)
add_dependencies(onrobot_gripper_msgs_generate_messages_cpp _onrobot_gripper_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(onrobot_gripper_msgs_gencpp)
add_dependencies(onrobot_gripper_msgs_gencpp onrobot_gripper_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS onrobot_gripper_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(onrobot_gripper_msgs
  "/home/robotica/ws_moveit/src/onrobot_gripper_msgs/msg/JointControllerState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/onrobot_gripper_msgs
)
_generate_msg_eus(onrobot_gripper_msgs
  "/home/robotica/ws_moveit/src/onrobot_gripper_msgs/msg/OnrobotGripperCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/onrobot_gripper_msgs
)
_generate_msg_eus(onrobot_gripper_msgs
  "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandAction.msg"
  "${MSG_I_FLAGS}"
  "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandResult.msg;/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandFeedback.msg;/home/robotica/ws_moveit/src/onrobot_gripper_msgs/msg/OnrobotGripperCommand.msg;/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandActionGoal.msg;/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandGoal.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandActionFeedback.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandActionResult.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/onrobot_gripper_msgs
)
_generate_msg_eus(onrobot_gripper_msgs
  "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandGoal.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/robotica/ws_moveit/src/onrobot_gripper_msgs/msg/OnrobotGripperCommand.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/onrobot_gripper_msgs
)
_generate_msg_eus(onrobot_gripper_msgs
  "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandActionResult.msg"
  "${MSG_I_FLAGS}"
  "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandResult.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/onrobot_gripper_msgs
)
_generate_msg_eus(onrobot_gripper_msgs
  "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/onrobot_gripper_msgs
)
_generate_msg_eus(onrobot_gripper_msgs
  "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/robotica/ws_moveit/src/onrobot_gripper_msgs/msg/OnrobotGripperCommand.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/onrobot_gripper_msgs
)
_generate_msg_eus(onrobot_gripper_msgs
  "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/onrobot_gripper_msgs
)
_generate_msg_eus(onrobot_gripper_msgs
  "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/onrobot_gripper_msgs
)

### Generating Services

### Generating Module File
_generate_module_eus(onrobot_gripper_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/onrobot_gripper_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(onrobot_gripper_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(onrobot_gripper_msgs_generate_messages onrobot_gripper_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/robotica/ws_moveit/src/onrobot_gripper_msgs/msg/JointControllerState.msg" NAME_WE)
add_dependencies(onrobot_gripper_msgs_generate_messages_eus _onrobot_gripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotica/ws_moveit/src/onrobot_gripper_msgs/msg/OnrobotGripperCommand.msg" NAME_WE)
add_dependencies(onrobot_gripper_msgs_generate_messages_eus _onrobot_gripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandAction.msg" NAME_WE)
add_dependencies(onrobot_gripper_msgs_generate_messages_eus _onrobot_gripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandActionGoal.msg" NAME_WE)
add_dependencies(onrobot_gripper_msgs_generate_messages_eus _onrobot_gripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandActionResult.msg" NAME_WE)
add_dependencies(onrobot_gripper_msgs_generate_messages_eus _onrobot_gripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandActionFeedback.msg" NAME_WE)
add_dependencies(onrobot_gripper_msgs_generate_messages_eus _onrobot_gripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandGoal.msg" NAME_WE)
add_dependencies(onrobot_gripper_msgs_generate_messages_eus _onrobot_gripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandResult.msg" NAME_WE)
add_dependencies(onrobot_gripper_msgs_generate_messages_eus _onrobot_gripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandFeedback.msg" NAME_WE)
add_dependencies(onrobot_gripper_msgs_generate_messages_eus _onrobot_gripper_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(onrobot_gripper_msgs_geneus)
add_dependencies(onrobot_gripper_msgs_geneus onrobot_gripper_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS onrobot_gripper_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(onrobot_gripper_msgs
  "/home/robotica/ws_moveit/src/onrobot_gripper_msgs/msg/JointControllerState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/onrobot_gripper_msgs
)
_generate_msg_lisp(onrobot_gripper_msgs
  "/home/robotica/ws_moveit/src/onrobot_gripper_msgs/msg/OnrobotGripperCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/onrobot_gripper_msgs
)
_generate_msg_lisp(onrobot_gripper_msgs
  "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandAction.msg"
  "${MSG_I_FLAGS}"
  "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandResult.msg;/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandFeedback.msg;/home/robotica/ws_moveit/src/onrobot_gripper_msgs/msg/OnrobotGripperCommand.msg;/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandActionGoal.msg;/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandGoal.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandActionFeedback.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandActionResult.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/onrobot_gripper_msgs
)
_generate_msg_lisp(onrobot_gripper_msgs
  "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandGoal.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/robotica/ws_moveit/src/onrobot_gripper_msgs/msg/OnrobotGripperCommand.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/onrobot_gripper_msgs
)
_generate_msg_lisp(onrobot_gripper_msgs
  "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandActionResult.msg"
  "${MSG_I_FLAGS}"
  "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandResult.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/onrobot_gripper_msgs
)
_generate_msg_lisp(onrobot_gripper_msgs
  "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/onrobot_gripper_msgs
)
_generate_msg_lisp(onrobot_gripper_msgs
  "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/robotica/ws_moveit/src/onrobot_gripper_msgs/msg/OnrobotGripperCommand.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/onrobot_gripper_msgs
)
_generate_msg_lisp(onrobot_gripper_msgs
  "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/onrobot_gripper_msgs
)
_generate_msg_lisp(onrobot_gripper_msgs
  "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/onrobot_gripper_msgs
)

### Generating Services

### Generating Module File
_generate_module_lisp(onrobot_gripper_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/onrobot_gripper_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(onrobot_gripper_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(onrobot_gripper_msgs_generate_messages onrobot_gripper_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/robotica/ws_moveit/src/onrobot_gripper_msgs/msg/JointControllerState.msg" NAME_WE)
add_dependencies(onrobot_gripper_msgs_generate_messages_lisp _onrobot_gripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotica/ws_moveit/src/onrobot_gripper_msgs/msg/OnrobotGripperCommand.msg" NAME_WE)
add_dependencies(onrobot_gripper_msgs_generate_messages_lisp _onrobot_gripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandAction.msg" NAME_WE)
add_dependencies(onrobot_gripper_msgs_generate_messages_lisp _onrobot_gripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandActionGoal.msg" NAME_WE)
add_dependencies(onrobot_gripper_msgs_generate_messages_lisp _onrobot_gripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandActionResult.msg" NAME_WE)
add_dependencies(onrobot_gripper_msgs_generate_messages_lisp _onrobot_gripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandActionFeedback.msg" NAME_WE)
add_dependencies(onrobot_gripper_msgs_generate_messages_lisp _onrobot_gripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandGoal.msg" NAME_WE)
add_dependencies(onrobot_gripper_msgs_generate_messages_lisp _onrobot_gripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandResult.msg" NAME_WE)
add_dependencies(onrobot_gripper_msgs_generate_messages_lisp _onrobot_gripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandFeedback.msg" NAME_WE)
add_dependencies(onrobot_gripper_msgs_generate_messages_lisp _onrobot_gripper_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(onrobot_gripper_msgs_genlisp)
add_dependencies(onrobot_gripper_msgs_genlisp onrobot_gripper_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS onrobot_gripper_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(onrobot_gripper_msgs
  "/home/robotica/ws_moveit/src/onrobot_gripper_msgs/msg/JointControllerState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/onrobot_gripper_msgs
)
_generate_msg_nodejs(onrobot_gripper_msgs
  "/home/robotica/ws_moveit/src/onrobot_gripper_msgs/msg/OnrobotGripperCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/onrobot_gripper_msgs
)
_generate_msg_nodejs(onrobot_gripper_msgs
  "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandAction.msg"
  "${MSG_I_FLAGS}"
  "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandResult.msg;/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandFeedback.msg;/home/robotica/ws_moveit/src/onrobot_gripper_msgs/msg/OnrobotGripperCommand.msg;/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandActionGoal.msg;/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandGoal.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandActionFeedback.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandActionResult.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/onrobot_gripper_msgs
)
_generate_msg_nodejs(onrobot_gripper_msgs
  "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandGoal.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/robotica/ws_moveit/src/onrobot_gripper_msgs/msg/OnrobotGripperCommand.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/onrobot_gripper_msgs
)
_generate_msg_nodejs(onrobot_gripper_msgs
  "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandActionResult.msg"
  "${MSG_I_FLAGS}"
  "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandResult.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/onrobot_gripper_msgs
)
_generate_msg_nodejs(onrobot_gripper_msgs
  "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/onrobot_gripper_msgs
)
_generate_msg_nodejs(onrobot_gripper_msgs
  "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/robotica/ws_moveit/src/onrobot_gripper_msgs/msg/OnrobotGripperCommand.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/onrobot_gripper_msgs
)
_generate_msg_nodejs(onrobot_gripper_msgs
  "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/onrobot_gripper_msgs
)
_generate_msg_nodejs(onrobot_gripper_msgs
  "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/onrobot_gripper_msgs
)

### Generating Services

### Generating Module File
_generate_module_nodejs(onrobot_gripper_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/onrobot_gripper_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(onrobot_gripper_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(onrobot_gripper_msgs_generate_messages onrobot_gripper_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/robotica/ws_moveit/src/onrobot_gripper_msgs/msg/JointControllerState.msg" NAME_WE)
add_dependencies(onrobot_gripper_msgs_generate_messages_nodejs _onrobot_gripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotica/ws_moveit/src/onrobot_gripper_msgs/msg/OnrobotGripperCommand.msg" NAME_WE)
add_dependencies(onrobot_gripper_msgs_generate_messages_nodejs _onrobot_gripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandAction.msg" NAME_WE)
add_dependencies(onrobot_gripper_msgs_generate_messages_nodejs _onrobot_gripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandActionGoal.msg" NAME_WE)
add_dependencies(onrobot_gripper_msgs_generate_messages_nodejs _onrobot_gripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandActionResult.msg" NAME_WE)
add_dependencies(onrobot_gripper_msgs_generate_messages_nodejs _onrobot_gripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandActionFeedback.msg" NAME_WE)
add_dependencies(onrobot_gripper_msgs_generate_messages_nodejs _onrobot_gripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandGoal.msg" NAME_WE)
add_dependencies(onrobot_gripper_msgs_generate_messages_nodejs _onrobot_gripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandResult.msg" NAME_WE)
add_dependencies(onrobot_gripper_msgs_generate_messages_nodejs _onrobot_gripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandFeedback.msg" NAME_WE)
add_dependencies(onrobot_gripper_msgs_generate_messages_nodejs _onrobot_gripper_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(onrobot_gripper_msgs_gennodejs)
add_dependencies(onrobot_gripper_msgs_gennodejs onrobot_gripper_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS onrobot_gripper_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(onrobot_gripper_msgs
  "/home/robotica/ws_moveit/src/onrobot_gripper_msgs/msg/JointControllerState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/onrobot_gripper_msgs
)
_generate_msg_py(onrobot_gripper_msgs
  "/home/robotica/ws_moveit/src/onrobot_gripper_msgs/msg/OnrobotGripperCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/onrobot_gripper_msgs
)
_generate_msg_py(onrobot_gripper_msgs
  "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandAction.msg"
  "${MSG_I_FLAGS}"
  "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandResult.msg;/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandFeedback.msg;/home/robotica/ws_moveit/src/onrobot_gripper_msgs/msg/OnrobotGripperCommand.msg;/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandActionGoal.msg;/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandGoal.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandActionFeedback.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandActionResult.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/onrobot_gripper_msgs
)
_generate_msg_py(onrobot_gripper_msgs
  "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandGoal.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/robotica/ws_moveit/src/onrobot_gripper_msgs/msg/OnrobotGripperCommand.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/onrobot_gripper_msgs
)
_generate_msg_py(onrobot_gripper_msgs
  "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandActionResult.msg"
  "${MSG_I_FLAGS}"
  "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandResult.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/onrobot_gripper_msgs
)
_generate_msg_py(onrobot_gripper_msgs
  "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/onrobot_gripper_msgs
)
_generate_msg_py(onrobot_gripper_msgs
  "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/robotica/ws_moveit/src/onrobot_gripper_msgs/msg/OnrobotGripperCommand.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/onrobot_gripper_msgs
)
_generate_msg_py(onrobot_gripper_msgs
  "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/onrobot_gripper_msgs
)
_generate_msg_py(onrobot_gripper_msgs
  "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/onrobot_gripper_msgs
)

### Generating Services

### Generating Module File
_generate_module_py(onrobot_gripper_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/onrobot_gripper_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(onrobot_gripper_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(onrobot_gripper_msgs_generate_messages onrobot_gripper_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/robotica/ws_moveit/src/onrobot_gripper_msgs/msg/JointControllerState.msg" NAME_WE)
add_dependencies(onrobot_gripper_msgs_generate_messages_py _onrobot_gripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotica/ws_moveit/src/onrobot_gripper_msgs/msg/OnrobotGripperCommand.msg" NAME_WE)
add_dependencies(onrobot_gripper_msgs_generate_messages_py _onrobot_gripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandAction.msg" NAME_WE)
add_dependencies(onrobot_gripper_msgs_generate_messages_py _onrobot_gripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandActionGoal.msg" NAME_WE)
add_dependencies(onrobot_gripper_msgs_generate_messages_py _onrobot_gripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandActionResult.msg" NAME_WE)
add_dependencies(onrobot_gripper_msgs_generate_messages_py _onrobot_gripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandActionFeedback.msg" NAME_WE)
add_dependencies(onrobot_gripper_msgs_generate_messages_py _onrobot_gripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandGoal.msg" NAME_WE)
add_dependencies(onrobot_gripper_msgs_generate_messages_py _onrobot_gripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandResult.msg" NAME_WE)
add_dependencies(onrobot_gripper_msgs_generate_messages_py _onrobot_gripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotica/ws_moveit/devel/.private/onrobot_gripper_msgs/share/onrobot_gripper_msgs/msg/OnrobotGripperCommandFeedback.msg" NAME_WE)
add_dependencies(onrobot_gripper_msgs_generate_messages_py _onrobot_gripper_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(onrobot_gripper_msgs_genpy)
add_dependencies(onrobot_gripper_msgs_genpy onrobot_gripper_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS onrobot_gripper_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/onrobot_gripper_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/onrobot_gripper_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET trajectory_msgs_generate_messages_cpp)
  add_dependencies(onrobot_gripper_msgs_generate_messages_cpp trajectory_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(onrobot_gripper_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(onrobot_gripper_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET actionlib_msgs_generate_messages_cpp)
  add_dependencies(onrobot_gripper_msgs_generate_messages_cpp actionlib_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/onrobot_gripper_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/onrobot_gripper_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET trajectory_msgs_generate_messages_eus)
  add_dependencies(onrobot_gripper_msgs_generate_messages_eus trajectory_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(onrobot_gripper_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(onrobot_gripper_msgs_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET actionlib_msgs_generate_messages_eus)
  add_dependencies(onrobot_gripper_msgs_generate_messages_eus actionlib_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/onrobot_gripper_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/onrobot_gripper_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET trajectory_msgs_generate_messages_lisp)
  add_dependencies(onrobot_gripper_msgs_generate_messages_lisp trajectory_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(onrobot_gripper_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(onrobot_gripper_msgs_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET actionlib_msgs_generate_messages_lisp)
  add_dependencies(onrobot_gripper_msgs_generate_messages_lisp actionlib_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/onrobot_gripper_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/onrobot_gripper_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET trajectory_msgs_generate_messages_nodejs)
  add_dependencies(onrobot_gripper_msgs_generate_messages_nodejs trajectory_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(onrobot_gripper_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(onrobot_gripper_msgs_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET actionlib_msgs_generate_messages_nodejs)
  add_dependencies(onrobot_gripper_msgs_generate_messages_nodejs actionlib_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/onrobot_gripper_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/onrobot_gripper_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/onrobot_gripper_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET trajectory_msgs_generate_messages_py)
  add_dependencies(onrobot_gripper_msgs_generate_messages_py trajectory_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(onrobot_gripper_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(onrobot_gripper_msgs_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET actionlib_msgs_generate_messages_py)
  add_dependencies(onrobot_gripper_msgs_generate_messages_py actionlib_msgs_generate_messages_py)
endif()
