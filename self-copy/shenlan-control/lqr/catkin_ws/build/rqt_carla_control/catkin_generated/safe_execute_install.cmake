execute_process(COMMAND "/home/hua/shenlan-control/lqr/catkin_ws/build/rqt_carla_control/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/hua/shenlan-control/lqr/catkin_ws/build/rqt_carla_control/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
