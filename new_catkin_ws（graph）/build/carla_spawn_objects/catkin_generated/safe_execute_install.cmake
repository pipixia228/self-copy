execute_process(COMMAND "/home/hua/new_catkin_ws/build/carla_spawn_objects/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/hua/new_catkin_ws/build/carla_spawn_objects/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
