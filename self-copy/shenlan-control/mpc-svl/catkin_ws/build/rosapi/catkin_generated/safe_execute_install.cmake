execute_process(COMMAND "/home/hua/shenlan-control/mpc/catkin_ws/build/rosapi/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/hua/shenlan-control/mpc/catkin_ws/build/rosapi/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
