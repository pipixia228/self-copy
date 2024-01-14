#pragma once
#include <math.h>
#include <fstream>
#include <iomanip>
#include <memory>
#include <string>

#include "Eigen/Core"
#include "common.h"

namespace hua
{
    namespace control
    {
        using Matrix = Eigen::MatrixXd;

        class LqrController
        {
        public:
            LqrController();
            ~LqrController();

            void LoadControlConf(); // 加载控制配置参数
            void Init();            // 初始化控制器

            bool ComputeControlCommand(
                const VehicleState &localization,
                const TrajectoryData &planning_published_trajectory, ControlCmd &cmd); // 计算控制指令

        protected:
            void UpdateState(const VehicleState &vehicle_state); // 更新车辆状态信息

            void UpdateMatrix(const VehicleState &vehicle_state); // 更新状态矩阵

            double ComputeFeedForward(const VehicleState &localization,
                                      const double ref_curvature); // 计算前馈控制量

            void ComputeLateralErrors(const double x, const double y, const double theta,
                                      const double linear_v, const double angular_v,
                                      const double linear_a,
                                      LateralControlErrorPtr &lat_con_err); // 计算横向误差

            TrajectoryPoint QueryNearestPointByPosition(const double x, const double y); // 根据给定坐标查询最近的轨迹点

            void SolveLQRProblem(const Matrix &A, const Matrix &B, const Matrix &Q,
                                 const Matrix &R, const double tolerance,
                                 const uint max_num_iteration, Matrix *ptr_K); // 求解LQR问题，计算增益矩阵

            std::vector<TrajectoryPoint> trajectory_points_; // 轨迹点列表

            // 以下参数与车辆物理相关
            // 控制时间间隔
            double ts_ = 0.0;
            // 前轮侧偏刚度
            double cf_ = 0.0;
            // 后轮侧偏刚度
            double cr_ = 0.0;
            // 前后轮中心距离
            double wheelbase_ = 0.0;
            // 车辆质量
            double mass_ = 0.0;
            // 前轮中心到质心的距离
            double lf_ = 0.0;
            // 后轮中心到质心的距离
            double lr_ = 0.0;
            // 车辆的旋转惯量
            double iz_ = 0.0;
            // 方向盘转角与轮胎转角之间的比例系数
            double steer_ratio_ = 0.0;
            // 最大方向盘转角
            double steer_single_direction_max_degree_ = 0.0;

            // 没有预瞄的状态数量，包括横向误差、横向误差速率、航向误差和航向误差速率
            const int basic_state_size_ = 4;
            // 车辆状态矩阵
            Eigen::MatrixXd matrix_a_;
            // 离散时间的车辆状态矩阵
            Eigen::MatrixXd matrix_ad_;
            // 控制矩阵
            Eigen::MatrixXd matrix_b_;
            // 离散时间的控制矩阵
            Eigen::MatrixXd matrix_bd_;
            // 增益矩阵
            Eigen::MatrixXd matrix_k_;
            // 控制权重矩阵
            Eigen::MatrixXd matrix_r_;
            // 状态权重矩阵
            Eigen::MatrixXd matrix_q_;
            // 更新后的状态权重矩阵
            Eigen::MatrixXd matrix_q_updated_;
            // 车辆状态矩阵系数
            Eigen::MatrixXd matrix_a_coeff_;
            // 4x1的状态矩阵
            Eigen::MatrixXd matrix_state_;

            // LQR求解器参数：迭代次数
            int lqr_max_iteration_ = 0;
            // LQR求解器参数：计算阈值
            double lqr_eps_ = 0.0;

            // 前视控制器（开启后可以改善大曲率轨迹的控制效果）
            bool enable_look_ahead_back_control_ = false;

            // 用于计算差分值以估计加速度/纵向加速度变化率
            double previous_lateral_acceleration_ = 0.0;

            double previous_heading_rate_ = 0.0;     // 前一个航向角速度
            double previous_ref_heading_rate_ = 0.0; // 前一个参考航向角速度

            double previous_heading_acceleration_ = 0.0;     // 前一个航向角加速度
            double previous_ref_heading_acceleration_ = 0.0; // 前一个参考航向角加速度

            // 用于记录的目的
            std::ofstream steer_log_file_; // 记录目的地的文件流

            const std::string name_; // 控制器的名称

            double query_relative_time_; // 查询相对时间

            double pre_steer_angle_ = 0.0; // 前一个方向盘转角

            double pre_steering_position_ = 0.0; // 前一个转向位置

            double minimum_speed_protection_ = 0.1; // 最小速度保护，用于防止低速时控制指令过大

            double current_trajectory_timestamp_ = -1.0; // 当前轨迹时间戳

            double init_vehicle_x_ = 0.0;       // 车辆初始位置 x 坐标
            double init_vehicle_y_ = 0.0;       // 车辆初始位置 y 坐标
            double init_vehicle_heading_ = 0.0; // 车辆初始航向角

            double low_speed_bound_ = 0.0;  // 低速边界，用于限制控制指令在低速时的大小
            double low_speed_window_ = 0.0; // 低速窗口，用于调整低速边界的大小

            double driving_orientation_ = 0.0; // 行驶方向，即车头朝向
            double steer_offset_ = 0.0;        // 方向盘偏移量

            // 添加的
            double ref_curv_; // 参考曲率，用于计算前馈控制量
        };

    }
}