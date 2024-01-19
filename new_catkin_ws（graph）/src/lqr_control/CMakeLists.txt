cmake_minimum_required(VERSION 3.0.2)
project(lqr_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
add_definitions("-Wall -g")
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs      # ROS消息包，包含几何相关的消息
  carla_msgs         # ROS消息包，包含carla相关的消息
  nav_msgs           # ROS消息包，包含导航相关的消息
  roscpp             # ROS C++库
  rospy              # ROS Python库
  sensor_msgs        # ROS消息包，包含传感器相关的消息
  std_msgs           # ROS消息包，包含标准消息类型
  tf                 # ROS库，提供坐标变换功能
  ros_viz_tools      # ROS可视化工具包
)

catkin_package(
  LIBRARIES serial_communication
  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs tf carla_msgs nav_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}   # 包含catkin软件包的头文件路径
  "/usr/include/eigen3"    # 包含Eigen库的头文件路径
)

link_directories(${catkin_LIBRARIES} lib)   # 包含catkin软件包的库文件路径和项目的lib目录

add_library(lqr_control
            src/lqr_controller.cpp
            src/reference_line.cpp
            src/pid_controller.cpp)
               

target_link_libraries(lqr_control ${catkin_LIBRARIES} VTSMapInterfaceCPP)   # 链接依赖库catkin_LIBRARIES和VTSMapInterfaceCPP到lqr_control库

add_executable(lqr_control_node src/main.cpp src/lqr_controller_node.cpp)   # 定义可执行文件lqr_control_node，并添加源文件到其中
target_link_libraries(lqr_control_node lqr_control)   # 链接lqr_control库到lqr_control_node可执行文件
