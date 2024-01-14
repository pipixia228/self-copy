#include "lqr_controller.h"

#include <algorithm>
#include <iomanip>
#include <utility>
#include <vector>

#include "Eigen/LU"
#include "math.h"

using namespace std;

namespace hua
{
    namespace lqr_controller
    {
        LqrController::LqrController() {}

        LqrController::~LqrController() {}

        // lqr配置
        void LqrController::LoadControlConf()
        {
            ts = 0.01; // 每隔0.01秒进行一次控制

            cf_ = 155494.663;                           // 左右轮前轮侧扁刚度之和
            cr_ = 155494.663;                           // 左右轮后轮侧扁刚度之和
            wheelbase_ = 2.852;                         // 轴距
            steer_ratio_ = 16;                          // 方向盘转角与轮胎转角之间的比之系数
            steer_single_direction_max_degree_ = 470.0; // 最大方向转角

            const double mass_fl = 520;                  // 左前悬质量
            const double mass_fr = 520;                  // 右前悬质量
            const double mass_rl = 520;                  // 左后悬质量
            const double mass_rr = 520;                  // 右后悬质量
            const double mass_front = mass_fl + mass_fr; // 前悬质量
            const double mass_rear = mass_rl + mass_rr;  // 后悬质量
            mass_ = mass_front + mass_rear;

            lf_ = wheelbase_ * (1.0 - mass_front / mass_); // 前轴到质心的距离
            lr_ = wheelbase_ * (1.0 - mass_rear / mass_);  // 后轴到质心的距离

            iz_ = lf_ * lf_ * mass_front + lr_ * lr_ * mass_rear; // 汽车转动惯量

            lqr_eps_ = 0.01;            // lqr迭代求解器精度
            lqr_max_interation_ = 1500; // lqr迭代次数

            return;
        }

        // 初始化控制器
        void LqrController::Init()
        {
            // 初始化矩阵
            const int matrix_size = basic_state_size_;
            matrix_a_ = Matrix::Zero(basic_state_size_, basic_state_size_);
            matrix_ad_ = Matrix::Zero(basic_state_size_, basic_state_size_); // matrix_ad_ 为 matrix_a_ 离散化的版本

            /*
            A matrix (Gear Drive)
            [0.0,        1.0,                                     0.0,                              0.0;
             0.0,        (-(c_f + c_r) / m) / v,                  (c_f + c_r) / m,                  (l_r * c_r - l_f * c_f) / m / v;
             0.0,        0.0,                                     0.0,                              1.0;
             0.0,        ((lr * cr - lf * cf) / i_z) / v,         (l_f * c_f - l_r * c_r) / i_z,    (-1.0 * (l_f^2 * c_f + l_r^2 * c_r) / i_z) / v;]
            */

            // 初始化A矩阵的常数项（与v无关的元素）
            matrix_a_(0, 1) = 1.0;
            matrix_a_(1, 2) = (cf_ + cr_) / mass_;
            matrix_a_(2, 3) = 1.0;
            matrix_a_(3, 2) = (lf_ * cf_ - lr_ * cr_) / iz_;

            // 初始化A矩阵的非常数项
            matrix_a_coeff_ = Matrix::Zero(matrix_size, matrix_size);
            matrix_a_coeff_(1, 1) = -(cf_ + cr_) / mass_;
            matrix_a_coeff_(1, 3) = (lr_ * cr_ - lf_ * cf_) / mass_;
            matrix_a_coeff_(3, 1) = (lr_ * cr_ - lf_ * cf_) / iz_;
            matrix_a_coeff_(3, 3) = -(lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / iz_;

            /**
             * b = [0.0, cf / m, 0.0, lf * lr / iz]^T
             */

            // 初始化B矩阵
            matrix_b_ = Matrix::Zero(basic_state_size_, 1);
            matrix_bd_ = Matrix::Zero(basic_state_size_, 1); // matrix_bd_ 为 matrix_b_ 离散化的版本
            matrix_b_(1, 0) = cf_ / mass_;
            matrix_b_(3, 0) = lf_ * cf_ / iz;
            matrix_bd_ = matrix_b_ * ts_; // 离散化B_d = B * dt 向前欧拉离散化方法

            // 状态向量
            matrix_state_ = Matrix::Zero(matrix_size, 1);
            // 反馈矩阵
            matrix_k_ = Matrix::Zero(1, matrix_size);
            // lqr cost function中 输入值u的权重
            matrix_r_ = Matrix::Identity(1, 1); // Identity 是线性代数中的一个概念，表示单位矩阵。该处表示一个1*1的单位矩阵
            matrix_r_(0, 0) = 10;
            // lqr cost function中 状态向量x的权重
            matrix_q_ = Matrix::Zero(matrix_size, matrix_size);

            // int q_param_size = 4;
            matrix_q_(0, 0) = 1; // lateral_error
            matrix_q_(1, 1) = 1; // lateral_error_rate
            matrix_q_(2, 2) = 1; // heading_error
            matrix_q_(3, 3) = 1; // heading__error_rate

            matrix_q_updated_ = matrix_q_; // 更新q矩阵

            return;
        }

        // 两点间距离
        double PointDistanceSquare(const TrajectoryPoint &point, const double x, const double y)
        {
            double dx = point.x - x;
            double dy = point.y - y;
            return dx * dx + dy * dy;
        }

        // 将角度(弧度制)归化到[-M_PI, M_PI]之间
        double NormalizeAngle(const double angle)
        {
            double a = std::fmod(angle + M_PI, 2.0 * M_PI);
            // 函数接受两个参数 numerator 和 denominator，返回 numerator 除以 denominator 的余数。
            // 这个余数的符号与 numerator 的符号相同
            if (a < 0.0)
            {
                a += (2.0 * M_PI);
            }
            return a - M_PI;
        }

        // 计算控制命令
        bool LqrController::ComputeControlCommand(
            const VehicleState &localization,
            const TrajectoryData &planning_published_trajectory, ControlCmd &cmd)
        {
            // 规划轨迹
            trajectory_points_ = planning_published_trajectory.trajectory_points;
            /**
            // A matrix (Gear Drive)
            // [0.0,        1.0,                                     0.0,                              0.0;
            //  0.0,        (-(c_f + c_r) / m) / v,                  (c_f + c_r) / m,                  (l_r * c_r - l_f * c_f) / m / v;
            //  0.0,        0.0,                                     0.0,                              1.0;
            //  0.0,        ((lr * cr - lf * cf) / i_z) / v,         (l_f * c_f - l_r * c_r) / i_z,    (-1.0 * (l_f^2 * c_f + l_r^2 * c_r) / i_z) / v;]
            */

            // 配置状态矩阵A
            double v_ = std::max(localization.velocity, minimum_speed_protection_);
            matrix_a_(1, 1) = matrix_a_coeff_(1, 1) / v_;
            matrix_a_(1, 3) = matrix_a_coeff_(1, 3) / v_;
            matrix_a_(3, 1) = matrix_a_coeff_(3, 1) / v_;
            matrix_a_(3, 3) = matrix_a_coeff_(3, 3) / v_;

            /**
             * b = [0.0, c_f / m, 0.0, l_f * c_f / i_z]^T
             */
            // to-do 02 动力矩阵B
            // matrix_bd_ = matrix_bd_;

            // 计算横向误差并且更新状态向量x
            UpdateState(localization);

            // 更新状态矩阵A并将状态矩阵A离散化
            UpdateMatrix(localization);

            // to-do 05 Solve Lqr Problem
            /**
             * "matrix_ad_"：表示系统的状态转移矩阵A。
             * "matrix_bd_"：表示系统的输入矩阵B。
             * "matrix_q_"：表示状态权重矩阵Q，用于定义对状态变量的重要性权重。
             * "matrix_r_"：表示输入权重矩阵R，用于定义对控制输入变量的重要性权重。
             * "lqr_eps_"：表示LQR算法的收敛条件，用于确定迭代的停止准则。
             * "lqr_max_iteration_"：表示LQR算法的最大迭代次数。
             * "&matrix_k_"：表示存储计算得到的最优控制增益矩阵K的变量。
             */
            SolveLQRProblem(matrix_ad_, matrix_bd_, matrix_q_, matrix_r_, lqr_eps_,
                            lqr_max_iteration_, &matrix_k_);

            // 求出最优控制率k, 算出反馈控制量 u = -k * x

            // 计算feedback
            //   feedback = - K * state
            //   Convert vehicle steer angle from rad to degree and then to steer degrees
            //   then to 100% ratio

            // 计算反馈控制的转向角度
            double steer_angle_feedback = 0.0;
            // matrix_k_表示控制增益矩阵K，matrix_state_表示系统状态向量。二者相乘得到的结果是一个1x1的矩阵，表示控制输入的值。
            //(0, 0)表示对该矩阵的第一行第一列进行索引操作，提取出该值，因为该矩阵只有一个元素。最后，将该值取负数，得到反馈控制的转向角度。
            steer_angle_feedback = -(matrix_k_ * matrix_state_)(0, 0);

            // 计算前馈控制，计算横向转角的反馈量
            double steer_angle_feedforward = 0.0;
            double feedforward_coef = 1.3;
            steer_angle_feedforward = ComputeFeedForward(localization, ref_curv_);

            // 组合反馈控制和前馈控制得到最终的转向角度
            double steer_angle = steer_angle_feedback + feedforward_coef * steer_angle_feedforward;

            // 限制前轮最大转角，这里定义前轮最大转角位于 [-20度～20度]
            double max_steer_angle = (double)20 * M_PI / 180;
            if (steer_angle >= max_steer_angle)
            {
                steer_angle = max_steer_angle;
            }
            else if (steer_angle <= -max_steer_angle)
            {
                steer_angle = -max_steer_angle;
            }
            cmd.steer_target = steer_angle;

            return true;
        }

        // 计算横向误差并更新状态向量x
        void LqrController::UpdateState(const VehicleState &vehicle_state)
        {
            // LateralControlError lat_con_err;  // 将其更改为智能指针,会自动在不再需要时释放内存，避免内存泄漏和悬挂指针等问题。
            std::shared_ptr<LateralControlError> lat_con_err =
                std::make_shared<LateralControlError>();

            // 计算横向误差
            ComputeLateralErrors(vehicle_state.x, vehicle_state.y, vehicle_state.heading,
                                 vehicle.velocity, vehicle_state.angular_velocity,
                                 vehicle_state.acceleration, lat_con_err);

            // 更新状态矩阵
            matrix_state_(0, 0) = lat_con_err->lateral_error;
            matrix_state_(1, 0) = lat_con_err->lateral_error_rate;
            matrix_state_(2, 0) = lat_con_err->heading_error;
            matrix_state_(3, 0) = lat_con_err->heading_error_rate;
        }

        // 更新状态矩阵A并离散化
        void LqrController::UpdateMatrix(const VehicleState &vehicle_state)
        {
            // 创建单位矩阵（与矩阵A大小一致）
            Matrix matrix_I = Matrix::Identity(matrix_a_.cols(), matrix_a_.rows());

            // 离散化Ad，使用中点欧拉法
            // 计算临时矩阵
            Matrix matrix_temp = (matrix_I - 0.5 * ts_ * matrix_a_).inverse();
            // 更新矩阵ad的值
            matrix_ad_ = matrix_temp * (matrix_I + 0.5 * ts_ * matrix_a_);
        }

        // 前馈控制，计算横向转角的反馈量
        double LqrController::ComputeFeedForward(const VehicleState &localization,
                                                 const double ref_curvature)
        {
            const double kv = lr_ * mass_ / 2 / cf_ / wheelbase_ - lf_ * mass_ / 2 / cr_ / wheelbase_;
            const double v = localization.velocity;
            double steer_angle_feedforwardterm = wheelbase_ * ref_curvature + kv * v * v * ref_curvature -
                                                 matrix_k_(0, 2) * (lr_ * ref_curvature -
                                                                    lf_ * mass_ * v * v * ref_curvature / 2 / cr_ / wheelbase_);
            return -steer_angle_feedforwardterm;
        }

        // 计算误差
        void LqrController::ComputeLateralErrors(const double x, const double y,
                                                 const double theta,
                                                 const double linear_v,
                                                 const double angular_v,
                                                 const double linear_a,
                                                 LateralControlErrorPtr &lat_con_err)
        {
            // 寻找匹配点
            TrajectoryPoint match_point = QueryNearestPointByPosition(x, y);

            // 计算横向误差
            const double dx = match_point.x - x;
            const double dy = match_point.y - y;

            // 误差计算公式（向量点乘）e_y = (x - x _m) * n_m = (dx, dy) * (-sin(theta_m), cos(theta_m))
            //  theta_m 为匹配点的heading
            const double cos_theta_m = std::cos(match_point.heading);
            const double sin_theta_m = std::sin(match_point.heading);

            // 计算横向误差
            double lateral_error = -dx * sin_theta_m + dy * cos_theta_m;
            lat_con_err->lateral_error = lateral_error;

            // 计算航向误差
            double heading_error = NormalizeAngle(match_point.heading - theta);

            lat_con_err->heading_error = heading_error;

            lat_con_err->lateral_error_rate = linear_v * std::sin(heading_error);
            lar_con_err->heading_error_rate = match_point.v * match_point.kappa - angular_v;
        }

        // 查询距离当前位置最近的轨迹点
        TrajectoryPoint LqrController::QueryNearestPointByPosition(const double x, const double y)
        {
            double d_min = PointDistanceSquare(trajectory_points_.front(), x, y);
            size_t index_min = 0;

            for (size_t i = 1; i < trajectory_points_.size(); ++i)
            {
                double d_temp = PointDistanceSquare(trajectory_points_[i], x, y);
                if (d_temp < d_min)
                {
                    d_min = d_temp;
                    index_min = i;
                }
            }

            ref_curv_ = trajectory_points_[index_min].kappa; // 对应的最近的轨迹点上的曲率

            return trajectory_points_[index_min];
        }

        // LQR方程求解
        void LqrController::SolveLQRProblem(const Matrix &A, const Matrix &B,
                                            const Matrix &Q, const Matrix &R,
                                            const double tolerance,
                                            const uint max_num_iteration,
                                            Matrix *ptr_K)
        {
            // 防止矩阵维数出错导致后续结果运算失败
            (A.rows() != A.cols() || B.rows() != A.rows() || Q.rows() != Q.cols() ||
             Q.rows() != A.rows() || R.rows() != R.cols() || R.rows() != B.cols())
            {
                std::cout
                    << "LQR solver: one or more matrices have incompatible dimensions."
                    << std::endl;
                return;
            }

            // 转置矩阵A、B
            Matrix A_trans = A.transpose();
            Matrix B_trans = B.transpose();

            //// 初始化状态权重矩阵P和收敛误差P_error
            /**
             * std::numeric_limits<double>::max() 是 C++ STL 中的一个函数，用于返回 double 类型的最大值。
             * 将其定义为一个极大值，以确保第一次迭代可以运行。随着迭代的进行，P_error 的值会逐渐减小，直到达到收敛误差的要求。
             */
            double P_error = std::numeric_limits<double>::max();
            Matrix P = Q;

            // 迭代计算状态权重矩阵
            uint num_iteration = 0;
            while (num_iteration++ <= max_num_iteration && P_error > tolerance)
            {
                // 计算下一步状态权重矩阵P_next
                Matrix P_next = Q + A_trans * P * A - A_trans * P * B * (R + B_trans * P * B).inverse() * B_trans * P * A;

                // 更新收敛误差P_error
                // P_next - P：计算矩阵 P_next 和矩阵 P 之间的差值。
                // .maxCoeff()：返回矩阵中的最大元素。
                // fabs()：取最大元素的绝对值。
                P_error = fabs((P_next - P).maxCoeff());

                // 更新状态权重矩阵P
                P = P_next;
            }

            // 计算控制器增益矩阵ptr_K
            *ptr_K = ((R + B_trans * P * B).inverse()) * B_trans * P * A;
        }
    }
}