#include "lqr_controller_node.h"

// 使用ROS参数服务器中的私有命名空间（~）来创建节点句柄
LQRControllerNode::LQRControllerNode() : pnh_("~") {}

// 析构函数为空，因为该节点没有需要清理的资源
LQRControllerNode::~LQRControllerNode() {}

// 初始化函数，用于读取配置参数、加载路网文件、初始化控制器等
bool LQRControllerNode::init()
{
    // 从参数服务器中读取配置参数
    std::string vehicle_odom_topic; // 车辆定位topic名
    std::string vehicle_cmd_topic;  // 车辆控制命令topic名
    std::string roadmap_path;       // 路网文件名
    std::string path_vis_topic;     // 可视化路网名
    std::string frame_id;           // 全局坐标系名
    double speed_P, speed_I, speed_D, target_speed, vis_frequency;

    pnh_.getParam("vehicle_odom_topic", vehicle_odom_topic); // 读取车辆定位话题名
    pnh_.getParam("vehicle_cmd_topic", vehicle_cmd_topic);   // 读取控制命令话题名
    pnh_.getParam("roadmap_path", roadmap_path);             // 读取路网文件名
    pnh_.getParam("path_vis_topic", path_vis_topic);         // 读取可视化路网名
    pnh_.getParam("target_speed", target_speed);             // 读取目标速度
    pnh_.getParam("goal_tolerance", goalTolerance_);         // 读取速度限
    pnh_.getParam("speed_P", speed_P);                       // 读取速度pid参数
    pnh_.getParam("speed_I", speed_I);
    pnh_.getParam("speed_D", speed_D);
    pnh_.getParam("control_frequency", controlFrequency_); // 读取控制频率
    pnh_.getParam("vis_frequency", vis_frequency);         // 读取路网显示频率
    pnh_.getParam("framed_id", frame_id);                  // 读取全局坐标系名

    // 加载路网文件
    if (!loadRoadmap(roadmap_path, target_speed))
        return false;

    // 初始化PID控制器
    speedPidControllerPtr_ = std::shared_ptr<PIDController>(
        new PIDController(speed_P, speed_I, speed_D));

    // 初始化LQR控制器
    lqrController_ = std::shared_ptr<LqrController>(new LqrController());
    lqrController_->LoadControlConf();
    lqrController_->Init();

    // 创建一个可视化工具类，用于路网可视化
    roadmapMakerPtr_ =
        std::shared_ptr<RosVizTools>(new RosVizTools(nh_, path_vis_topic));

    // 创建订阅器，接受车辆定位数据
    VehiclePoseSub_ = nh.subscribe(vehicle_odom_topic, 10, &LQRControllerNode::odomCallback, this);

    // 创建发布器。发布车辆控制命令
    controlPub_ = nh.advertise<carla_msgs::CarlaEgoVehicleControl>(vehicle_cmd_topic, 1000);

    // 创建定时器，用于路网可视化
    visTimer_ = nh.creatTimer(ros::Duration(1 / vis_frequency), &LQRControllerNode::visTimerLoop, this);

    // 创建定时器，用于控制
    controlTimer_ nh.creatTimer(ros::Duration(1 / controlFrequency_), &LQRControllerNode::controlTimerLoop, this);

    // 将规划轨迹加入路网可视化
    addRoadmapMarker(planningPublishedTrajectory_.trajectory_points, frame_id);

    // 确定目标点
    goalPoint_ = planningPublishedTrajectory_.trajectory_points.back();

    ROS_INO("lqr_control_node init finish");
    return true;
}

double LQRControllerNode::pid_control()
{
    double ego_speed = vehicleState_.velocity; // 当前车速
    double v_err = targetSpeed_ - ego_speed;   // 目标车速和当前车速的误差
    cout << "v_err: " << v_err << endl;
    cout << "targetSpeed_ is " << targetSpeed_ << endl;

    double acceleration_cmd = speedPidControllerPtr_->Control(v_err, 1 / controlFrequency_);
    return acceleration_cmd;
}

void LQRControllerNode::odomCallback(const nav_msgs::Odometry : ConstPtr &msg)
{
    // 如果是第一次接收到里程计数据，则记录车辆的初始位置
    if (firstRecord_)
    {
        vehicleState_.planning_init_x = msg->pose.pose.position.x;
        vehicleState_.planning_init_y = msg->pose.pose.position.y;
        firstRecord_ = false;
    }

    // 将当前位置信息存储到vehicleState_对象中
    vehicleState_.x = msg->pose.pose.position.x;
    vehicleState_.y = msg->pose.pose.position.y;

    // 将orientation(四元数)转换为欧拉角（roll,pitch,yaw）
    tf::Quaternion q;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
    tf::Matrix3x3(q).getRPY(vehicleState_.roll, vehicleState_.pitch,
                            vehicleState_.yaw);

    // vehicleState_.heading 表示车辆的航向角（yaw）
    vehicleState_.heading = vehicleState_.yaw;

    vehicleState_.velocity = // 速度
        std::sqrt(msg->twist.twist.linear.x * msg->twist.twist.linear.x +
                  msg->twist.twist.linear.y * msg->twist.twist.linear.y);
    vehicleState_.angular_velocity = // 转角速度
        std::sqrt(msg->twist.twist.angular.x * msg->twist.twist.angular.x +
                  msg->twist.twist.angular.y * msg->twist.twist.angular.y);
    vehicleState_.acceleration = 0.0; // 加速度
}
