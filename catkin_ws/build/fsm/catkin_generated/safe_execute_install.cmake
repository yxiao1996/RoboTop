execute_process(COMMAND "/home/yxiao1996/workspace/RoboTop/catkin_ws/build/fsm/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/yxiao1996/workspace/RoboTop/catkin_ws/build/fsm/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
