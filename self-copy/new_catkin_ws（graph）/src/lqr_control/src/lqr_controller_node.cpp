#include "lqr_controller_node.h"

#include <fstream>

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
    roadmapMarkerPtr_ =
        std::shared_ptr<RosVizTools>(new RosVizTools(nh_, path_vis_topic));

    // 创建订阅器，接受车辆定位数据
    VehiclePoseSub_ = nh_.subscribe(vehicle_odom_topic, 10, &LQRControllerNode::odomCallback, this);

    // 创建发布器。发布车辆控制命令
    controlPub_ = nh_.advertise<carla_msgs::CarlaEgoVehicleControl>(vehicle_cmd_topic, 1000);

    // 创建定时器，用于路网可视化
    visTimer_ = nh_.createTimer(ros::Duration(1 / vis_frequency), &LQRControllerNode::visTimerLoop, this);

    // 创建定时器，用于控制
    controlTimer_ = nh_.createTimer(ros::Duration(1 / controlFrequency_), &LQRControllerNode::controlTimerLoop, this);

    // 将规划轨迹加入路网可视化
    addRoadmapMarker(planningPublishedTrajectory_.trajectory_points, frame_id);

    // 确定目标点
    goalPoint_ = planningPublishedTrajectory_.trajectory_points.back();

    ROS_INFO("lqr_control_node init finish!");
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

void LQRControllerNode::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
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

bool LQRControllerNode::loadRoadmap(const std::string &roadmap_path,
                                    const double target_speed)
{
    // 读取参考线路径
    std::ifstream infile;
    infile.open(roadmap_path); // 连接文件流与文件
    if (!infile.is_open())
    {
        return false;
    }

    std::vector<std::pair<double, double>> xy_points; // 存储读取到的路径点坐标
    std::string s, x, y;                              // 存储读取的字符串和坐标值
    while (getline(infile, s))                        // 按行读取文件内容
    {
        std::stringstream word(s); // 从当前读取的行中创建文件流对象
        word >> x;
        word >> y;
        // 将读取的坐标值转换为double类型
        double pt_x = std::atof(x.c_str());
        double pt_y = std::atof(y.c_str());
        xy_points.push_back(std::make_pair(pt_x, pt_y)); // 将读取的坐标值u保存到vector中
    }
    infile.close(); // 关闭文件

    // 构建参考路径的配置文件
    using namespace hua::control;
    std::vector<double> headings, accumulated_s, kappas, dkappas;

    // 根据离散点组成的路径，生成路网航向角，累计距离，曲率，曲率倒数
    std::unique_ptr<ReferenceLine> reference_line =
        std::make_unique<ReferenceLine>(xy_points);
    reference_line->ComputePathProfile(&headings, &accumulated_s, &kappas, &dkappas);

    // 将每个路径点的信息构造成TrajectoryPoint对象，并添加到planningPublishedTrajectory_.trajectory_points中
    for (size_t i = 0; i < headings.size(); i++)
    {
        TrajectoryPoint trajectory_pt;
        trajectory_pt.x = xy_points[i].first;                                    // 路径点x坐标
        trajectory_pt.y = xy_points[i].second;                                   // 路径点y坐标
        trajectory_pt.v = target_speed;                                          // 路径点速度
        trajectory_pt.a = 0.0;                                                   // 路径点加速度
        trajectory_pt.heading = headings[i];                                     // 路径点航向角
        trajectory_pt.kappa = kappas[i];                                         // 路径点曲率
        planningPublishedTrajectory_.trajectory_points.push_back(trajectory_pt); // 将构造好的路径点添加到现有轨迹中
    }

    return true; // 成功加载
}

void LQRControllerNode::addRoadmapMarker(
    const std::vector<TrajectoryPoint> &path, const std::string &frame_id)
{
    roadmapMarkerPtr_->clear();        // 清空可视化路网消息
    std::string ns = "reference_path"; // 可视化消息的namespace
    // 创建一个新的线条标记，用于可视化参考路径
    visualization_msgs::Marker marker_linestrip = RosVizTools::newLineStrip(
        0.5, ns, 0, ros_viz_tools::LIGHT_BLUE, frame_id);
    /**
     * 0.5：线条的宽度，这里设置为0.5。
     * ns：可视化消息的命名空间，这里使用了之前定义的ns变量，其中存储了命名空间的名称。
     * 0：线条的ID，用于标识不同的可视化消息对象，这里设置为0。
     * ros_viz_tools::LIGHT_BLUE：线条的颜色，ros_viz_tools是一个命名空间，LIGHT_BLUE是其中定义的一个颜色常量，表示浅蓝色。
     * frame_id：线条所属的坐标系，用于确定线条在哪个坐标系下进行绘制。
     */

    // 将路径点添加到线条标记的points字段
    for (auto path_point : path)
    {
        // 遍历路径点
        geometry_msgs::Point p;
        p.x = path_point.x;
        p.y = path_point.y;
        p.z = 0;
        marker_linestrip.points.push_back(p);
    }
    std::cout << "path size is " << marker_linestrip.points.size() << std::endl;
    roadmapMarkerPtr_->append(marker_linestrip);
    return;
}

void LQRControllerNode::visTimerLoop(const ros::TimerEvent &)
{
    // 定时器回调函数
    roadmapMarkerPtr_->publish();
}

void LQRControllerNode::controlTimerLoop(const ros::TimerEvent &)
{
    ControlCmd cmd; // 控制指令对象
    if (!firstRecord_)
    {
        // 若车辆与目标点的距离小于设定距离，将目标速度设置为0、设置isReachGoal_为true
        if (pointDistance(goalPoint_, vehicleState_.x, vehicleState_.y) < goalTolerance_)
        {
            targetSpeed_ = 0;
            isReachGoal_ = true;
        }

        if (!isReachGoal_)
        {
            // 未达到目标点则使用LQR控制器计算控制命令
            lqrController_->ComputeControlCommand(vehicleState_, planningPublishedTrajectory_, cmd);
        }

        carla_msgs::CarlaEgoVehicleControl control_cmd; // 车辆控制指令对象
        control_cmd.header.stamp = ros::Time::now();    // 设置控制时间戳
        control_cmd.reverse = false;                    // 设置是否倒车
        control_cmd.manual_gear_shift = false;          // 设置是否手动换档
        control_cmd.hand_brake = false;                 // 设置是否手刹
        control_cmd.gear = 0;                           // 设置档位

        // 纵向控制
        double acc_cmd = pid_control();

        // 根据纵向控制指令更新油门和刹车
        if (acc_cmd >= 0)
        {
            control_cmd.throttle = min(1.0, acc_cmd); // 若控制指令大于等于0，将油门置为1.0
            control_cmd.brake = 0.0;                  // 刹车为0
        }
        else
        {
            control_cmd.throttle = 0.0;             // 油门为0
            control_cmd.brake = min(1.0, -acc_cmd); // 若控制指令小于0，刹车置为1.0
        }

        if (targetSpeed_ == 0)
        {
            control_cmd.throttle = 0.0; // 速度为0则不需要油门控制
        }

        // 横向控制
        control_cmd.steer = cmd.steer_target; // 将LQR控制器计算的横向控制指令更新到控制指令对象的steer字段

        controlPub_.publish(control_cmd); // 发布控制指令到ROS话题
    }
}