execute_process(COMMAND "/home/robotica/ws_moveit/build/ur_kinematics/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/robotica/ws_moveit/build/ur_kinematics/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
