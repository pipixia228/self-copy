# CMake generated Testfile for 
# Source directory: /home/hua/shenlan-control/stanley/pro2_stanley/src/carla-ros-bridge/carla_waypoint_publisher
# Build directory: /home/hua/shenlan-control/stanley/pro2_stanley/build/carla_waypoint_publisher
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_carla_waypoint_publisher_roslaunch-check_launch "/home/hua/shenlan-control/stanley/pro2_stanley/build/carla_waypoint_publisher/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/hua/shenlan-control/stanley/pro2_stanley/build/carla_waypoint_publisher/test_results/carla_waypoint_publisher/roslaunch-check_launch.xml" "--return-code" "/usr/bin/cmake -E make_directory /home/hua/shenlan-control/stanley/pro2_stanley/build/carla_waypoint_publisher/test_results/carla_waypoint_publisher" "/opt/ros/noetic/share/roslaunch/cmake/../scripts/roslaunch-check -o \"/home/hua/shenlan-control/stanley/pro2_stanley/build/carla_waypoint_publisher/test_results/carla_waypoint_publisher/roslaunch-check_launch.xml\" \"/home/hua/shenlan-control/stanley/pro2_stanley/src/carla-ros-bridge/carla_waypoint_publisher/launch\" ")
set_tests_properties(_ctest_carla_waypoint_publisher_roslaunch-check_launch PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/roslaunch/cmake/roslaunch-extras.cmake;66;catkin_run_tests_target;/home/hua/shenlan-control/stanley/pro2_stanley/src/carla-ros-bridge/carla_waypoint_publisher/CMakeLists.txt;15;roslaunch_add_file_check;/home/hua/shenlan-control/stanley/pro2_stanley/src/carla-ros-bridge/carla_waypoint_publisher/CMakeLists.txt;0;")
subdirs("gtest")
