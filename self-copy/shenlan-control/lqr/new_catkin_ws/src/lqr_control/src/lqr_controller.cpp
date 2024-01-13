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
    }
}