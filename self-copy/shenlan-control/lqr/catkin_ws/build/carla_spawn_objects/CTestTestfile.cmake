# CMake generated Testfile for 
# Source directory: /home/hua/shenlan-control/lqr/catkin_ws/src/carla-ros-bridge/carla_spawn_objects
# Build directory: /home/hua/shenlan-control/lqr/catkin_ws/build/carla_spawn_objects
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_carla_spawn_objects_roslaunch-check_launch "/home/hua/shenlan-control/lqr/catkin_ws/build/carla_spawn_objects/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/hua/shenlan-control/lqr/catkin_ws/build/carla_spawn_objects/test_results/carla_spawn_objects/roslaunch-check_launch.xml" "--return-code" "/usr/bin/cmake -E make_directory /home/hua/shenlan-control/lqr/catkin_ws/build/carla_spawn_objects/test_results/carla_spawn_objects" "/opt/ros/noetic/share/roslaunch/cmake/../scripts/roslaunch-check -o \"/home/hua/shenlan-control/lqr/catkin_ws/build/carla_spawn_objects/test_results/carla_spawn_objects/roslaunch-check_launch.xml\" \"/home/hua/shenlan-control/lqr/catkin_ws/src/carla-ros-bridge/carla_spawn_objects/launch\" ")
set_tests_properties(_ctest_carla_spawn_objects_roslaunch-check_launch PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/roslaunch/cmake/roslaunch-extras.cmake;66;catkin_run_tests_target;/home/hua/shenlan-control/lqr/catkin_ws/src/carla-ros-bridge/carla_spawn_objects/CMakeLists.txt;14;roslaunch_add_file_check;/home/hua/shenlan-control/lqr/catkin_ws/src/carla-ros-bridge/carla_spawn_objects/CMakeLists.txt;0;")
subdirs("gtest")
