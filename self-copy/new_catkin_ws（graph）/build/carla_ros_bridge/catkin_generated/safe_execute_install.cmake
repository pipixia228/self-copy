execute_process(COMMAND "/home/hua/文档/new_catkin_ws（graph）/build/carla_ros_bridge/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/hua/文档/new_catkin_ws（graph）/build/carla_ros_bridge/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
