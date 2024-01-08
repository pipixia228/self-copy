#include <fstream>
#include <iostream>
#include <string>

#include <carla_msgs/CarlaEgoVehicleControl.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Header.h>

#include "map.h"
#include "reference_line.h"
#include "ros/ros.h"
#include "tf/tf.h"

struct VehicleState
{
    double x;
    double y;
    double heading;          // 车辆朝向
    double kappa;            // 曲率(切线斜率)
    double velocity;         // 速度
    double angular_velocity; // 角速度
    double acceleration;     // 加速度

    // 规划起点
    double planning_init_x;
    double planning_init_y;

    double roll;
    double pitch;
    double yaw;

    double target_curv; // 目标点曲率

    double vx; // 车辆速度的x分量
    double vy; // 车辆速度的y分量

    // added
    double start_point_x; // 起始点的x坐标
    double start_point_y; // 起始点的y坐标

    double relative_x = 0; // 相对于参考点的x坐标（默认初始化为0）
    double relative_y = 0; // 相对于参考点的y坐标（默认初始化为0）

    double relative_distance = 0; // 相对于参考点的距离
};

// 轨迹点
struct TrajectoryPoint
{
    double x;       // x坐标
    double y;       // y坐标
    double heading; // 方向角
    double kappa;   // 曲率
    double v;       // 速度
    double a;       // 加速度
};

// 轨迹数据
struct TrajectoryData
{
    std::vector<TrajectoryPoint> trajectory_points; // 轨迹点集合
};

// 横向控制误差
struct LateralControlError
{
    double lateral_error;      // 横向误差
    double heading_error;      // 转向误差
    double lateral_error_rate; // 横向误差速率
    double heading_error_rate; // 转向误差速度
};

// 控制命令
struct ControlCmd
{
    double steer_target; // 转向目标
    double acc;          // 加速度
};

// 欧拉角
struct EulerAngles
{
    double roll;  // 横滚角
    double pitch; // 俯仰角
    double yaw;   // 偏航角
};

/**
 * LateralControlErrorPtr是一个类型别名，它定义了一个指向LateralControlError结构体的
 * std::shared_ptr共享指针类型。使用LateralControlErrorPtr可以方便地管理LateralControlError
 * 结构体对象的生命周期，实现动态内存分配和释放，以及实现多个指针共享同一个对象。
 * 这可以帮助减少内存泄漏和悬空指针等问题，并提高代码的安全性和可靠性。
*/
typedef std::shared_ptr<LateralControlError> LateralControlErrorPtr;
