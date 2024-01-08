/*****************************************************************
 * @file   map.h
 * @brief  An OpenDrive-based HD-Map Data Interface oriented to autonomous vehicle route planning.
 * @details This interface dedicats to provide autonomous vehicles' navigation module access to the necessary map data and also the
 * 			basic route planning function.
 *
 *          该文件是一个基于OpenDrive的高精度地图数据接口，面向自动驾驶车辆的路线规划。
 *          该接口旨在为自动驾驶车辆的导航模块提供访问所需的地图数据和基本的路线规划功能。
 *
 * @author zhonghua Hao
 * @version V1.0
 * @date   January 2024
 ***********************************************************************/
#pragma once
#ifndef MAP_H
#define MAP_H

#include <functional> // 引入函数对象的头文件
#include <vector>     // 引入向量容器的头文件
#include <string>     // 引入字符串处理的头文件
#include <cfloat>     // 引入浮点数宏定义的头文件

#ifdef _WIN32
#ifdef CPP_DLL_EXPORT
#define MAP_API __declspec(dllexport) // 定义导出符号
#else
// #define MAP_API __declspec(dllimport)
#define MAP_API // 定义空的导出符号，表示导入
#endif

#elif __linux__
#define MAP_API // 在Linux平台上定义空的导出符号

#endif

/**
 * @namespace zjlmap
 *
 * @brief All interfaces based on the ZJLab defined HDMap data.
 */
namespace zjlmap
{

    // ------------------------------------ Types redefined----------------------------------------------
    typedef double RoadS; // 路段上某一位置到道路起点的距离，沿着参考线测量
    /**
     * 路由上某一位置到路由起点（路由起点是指在路由（即规划的行驶路径）上的起始位置。
     * 它表示从起点位置开始沿着路由行驶的距离，通常使用RouteS类型来表示。路由起点可以是一个具体的位置点，
     * 也可以是某个道路上的特定位置。在路线规划中，将起点位置作为路由的起点，以确定行驶路径和导航方向。）的距离，沿着参考线测量
     */
    typedef double RouteS;

    /**
     * AnchorId：锚点的全局ID用于标识地图中的参考点，帮助确定其他地图实体的位置和方向。
     * RoadId：道路的全局ID用于唯一标识地图中的道路，可以用于查找、索引和引用特定的道路。
     * JunctionId：路口的全局ID用于标识地图中的交叉口，可以帮助导航系统准确计算车辆在路口的行驶行为和路径选择。
     * LocalLaneId：道路上车道的本地ID用于区分不同车道，每个车道都有一个唯一的本地ID，有助于车辆在特定车道上行驶。
     * ObjectId：对象的全局ID用于标识地图中的其他实体，如交通信号灯、停车场、POI等，以便进行查找和引用。
     * TrafficLightId：交通灯的全局ID用于唯一标识地图中的交通信号灯，以便控制和管理交通信号灯的状态和行为。
     * TrafficLightGroupId：交通灯组的全局ID用于标识地图中的交通信号灯组，可以将多个交通灯视为一个组，以便进行协调和同步控制。
     */
    typedef const char *AnchorId; // 锚点的全局ID

    typedef int32_t RoadId;      // 道路的全局ID
    typedef int32_t JunctionId;  // 路口的全局ID
    typedef int32_t LocalLaneId; // 道路上车道的本地ID

    typedef std::string ObjectId;            // 对象的全局ID
    typedef std::string TrafficLightId;      // 交通灯的全局ID
    typedef std::string TrafficLightGroupId; // 交通灯组的全局ID
    typedef std::string TrafficLightType;    // 交通灯的类型

    /**
     * @struct LaneId
     * @brief 表示车道的唯一标识
     */
    struct MAP_API LaneId
    {
        RoadId road_id;       // 道路的全局ID，用于标识所属道路
        int section_idx;      // 道路上的段索引，用于区分不同的道路段
        LocalLaneId local_id; // 车道在道路上的本地ID，用于唯一标识车道

        LaneId();                                  // 默认构造函数
        LaneId(RoadId ri, int si, LocalLaneId li); // 带参数的构造函数，用于初始化各个成员变量
        bool operator==(LaneId op);                // 重载 == 运算符，用于判断两个 LaneId 是否相等
    };

    /**
     * @struct LaneIdArray
     * @brief 表示车道ID数组
     */
    struct MAP_API LaneIdArray
    {
        uint32_t length;       // 车道ID数组的长度
        LaneId *lane_id_array; // 指向 LaneId 结构体的指针，存储实际的车道ID数组

        LaneIdArray(); // 默认构造函数
    };

    // ------------------------------------ Basic structures ---------------------------------------------------
    /**
     * @struct XYZ
     * @brief 表示某个位置的UTM XYZ坐标
     */
    struct MAP_API XYZ
    {
        double x;                             // UTM东向值
        double y;                             // UTM北向值（基于北半球的地图）
        double z;                             // 绝对高度值
        XYZ();                                // 默认构造函数
        XYZ(double xx, double yy, double zz); // 带参数的构造函数，用于初始化x、y、z值
    };

    /**
     * @struct XYZArray
     * @brief 表示XYZ位置数组
     */
    struct MAP_API XYZArray
    {
        uint32_t length;                 // XYZ位置数组的长度
        XYZ *xyz_array;                  // 指向XYZ结构体的指针，存储实际的XYZ位置数组
        XYZ operator[](int index) const; // 重载[]运算符，用于获取指定索引的XYZ位置
        XYZArray();                      // 默认构造函数
        ~XYZArray();                     // 析构函数，用于释放内存
    };

    /**
     * @struct SLZ
     * @brief 基于道路参考线的道路坐标
     */
    struct MAP_API SLZ
    {
        LaneId lane_id;                                  // 全局车道ID
        RoadS s;                                         // 基于道路参考线的s值
        double l;                                        // 相对于道路参考线的l值偏移量
        double z;                                        // 基于参考线平面的z值
        SLZ();                                           // 默认构造函数
        SLZ(LaneId id, double ss, double ll, double zz); // 带参数的构造函数，用于初始化各个成员变量
    };

    /**
     * @struct SLZArray
     * @brief 表示SLZ位置数组
     */
    struct MAP_API SLZArray
    {
        uint32_t length;                 // SLZ位置数组的长度
        SLZ *slz_array;                  // 指向SLZ结构体的指针，存储实际的SLZ位置数组
        SLZ operator[](int index) const; // 重载[]运算符，用于获取指定索引的SLZ位置
        SLZArray();                      // 默认构造函数
        SLZArray(unsigned int size);     // 带参数的构造函数，用于初始化指定长度的SLZ位置数组
        ~SLZArray();                     // 析构函数，用于释放内存
    };

    /**
     * @struct Anchor
     * @brief 地图上具有特殊含义的点
     */
    struct MAP_API Anchor
    {
        AnchorId id;         // 表示锚点的字符串
        SLZ slz;             // 锚点的位置
        Anchor();            // 默认构造函数
        ~Anchor() = default; // 默认析构函数
    };

    /**
     * @struct AnchorArray
     * @brief 锚点数组
     */
    struct MAP_API AnchorArray
    {
        uint32_t length;                    // 锚点数组的长度
        Anchor *anchor_array;               // 指向Anchor结构体的指针，存储实际的锚点数组
        AnchorArray();                      // 默认构造函数
        Anchor operator[](int index) const; // 重载[]运算符，用于获取指定索引的锚点
        ~AnchorArray();                     // 析构函数，用于释放内存
    };

    // ------------------------------------ Lane related structures / enumration ----------------------------------------------
    /**
     * @enum LaneChangeType
     * @brief 枚举了车道上可能的变道选项
     */
    typedef enum
    {
        kNoChange = 0,          // 只能沿着该车道直行
        kLeftChange = 1,        // 向左变道
        kRightChange = 2,       // 向右变道
        kBothChange = 4,        // 同时可以向左和向右变道
        kUnknownChangeType = -1 // 未知的变道类型

    } LaneChangeType;

    /**
     * @enum SingleLaneRoadMarkType
     * @brief 枚举了车道标线的类型（如果标线有双线，则选择离车道中心最近的那一条）
     */
    typedef enum
    {
        kUnknownMark = 0,  // 未知类型
        kBrokenWhite = 1,  // 虚线白色
        kBrokenYellow = 2, // 虚线黄色
        kSolidWhite = 3,   // 实线白色
        kSolidYellow = 4   // 实线黄色

    } SingleLaneRoadMarkType;

    /**
     * @enum LaneTurnType
     * @brief 枚举了车道的转向类型，目前只有前四个枚举值有效
     */
    typedef enum LaneTurnType_
    {
        kNoturn,          // 非路口或者断头路
        kForward,         // 直行
        kTurnLeft,        // 左转
        kTurnRight,       // 右转
        kForwardAndLeft,  // 直行加左转
        kForwardAndRight, // 直行加右转
        KLeftAndRight,    // 左右转
        kUTurn,           // U形弯
        KAll              // 直行和左右转皆可
    } LaneTurnType;

    /**
     * @enum LaneTurnType
     * @brief 枚举了车道的转向类型，可以通过位运算来表示多个转向类型的组合
     *      未知 kUnkonwnTurnType	= -1,
            非路口 KNOTURN				= 0X0,
            直行 KFORWARD				= 0X1,
            左转 KLEFTTURN 				= 0X2,
            右转 KRIGHTTURN 				= 0X4,
            u型弯 KUTURN 				= 0X8,

            叠加型举例：
            直行加左转：                  = 0x3
            直行加右转					= 0x5
            仅左右转						= 0x6
            所有皆可						= 0xF
    */
    typedef int32_t LaneTurnTypeOrigin; // 注意：标准OpenDrive地图中没有转向类型属性

    /**
     * @struct LaneInfo
     * @brief 描述车道导航所需的必要信息
     *
     */
    struct MAP_API LaneInfo
    {
        LaneId id;    // 车道的全局ID
        RoadS begin;  // 车道在所属道路上的起始位置
        RoadS end;    // 车道在所属道路上的结束位置
        RoadS length; // 车道的长度
        // LaneTurnType lane_turn_type; // 注意：目前地图中不包含转向类型属性，因此您读取的所有转向类型都将是 "kUnkonwnTurnType"
        LaneInfo();
    };

    struct MAP_API LaneLinkage
    {

        std::vector<LaneId> predecessor_lanes; // 前驱车道的ID列表
        std::vector<LaneId> successor_lanes;   // 后继车道的ID列表

        int left_neighbor_valid;  // 左邻车道是否有效，-1表示逆向，0表示无效，1表示同向
        int right_neighbor_valid; // 右邻车道是否有效，-1表示逆向，0表示无效，1表示同向
        LaneId left_neighbor;     // 左邻车道的ID
        LaneId right_neighbor;    // 右邻车道的ID

        LaneLinkage() = default;
    };

    // ------------------------------------ Object related structures / enumration ----------------------------------------------

    /**
     * @enum ObjectType
     * @brief 枚举了与路线规划相关的OpenDrive对象类型
     */
    typedef const char *ObjectType;

    /**
     * @struct ObjectTypeArray
     * @brief 存储ObjectType的数组结构
     */
    struct MAP_API ObjectTypeArray
    {
        uint32_t length;        // 对象类型数组的长度
        SLZ *object_type_array; // 指向对象类型数组的指针
    };

    /**
     *  @struct ObjectInRoute
     *  @brief 轻量级交通对象信息结构体，包含了对象ID、类型、距离起点的距离、坐标等信息
     *  @warning 目前该结构体尚未被使用
     */
    struct MAP_API ObjectInRoute
    {
        ObjectId id;            // 对象ID
        ObjectType type;        // 对象类型
        RouteS distance;        // 距离起点的距离
        SLZArray corner_points; // 对象的坐标
        double height;          // 对象高度

        ObjectInRoute();
    };

    /**
     * @struct ObjectInRouteArray
     * @brief 存储ObjectInRoute的数组结构
     * @warning 目前该结构体尚未被使用
     */
    struct MAP_API ObjectInRouteArray
    {
        uint32_t length;                      // ObjectInRoute数组的长度
        ObjectInRoute *object_in_route_array; // 指向ObjectInRoute数组的指针

        ObjectInRouteArray();
    };

    /**
     * @struct ObjectOnRoad
     * @brief 存储车道上的交通对象的结构体，包括对象ID、所在位置、类型、坐标等信息
     * @warning 目前该结构体尚未被使用
     */
    struct MAP_API ObjectOnRoad
    {
        ObjectId id;            // 对象ID
        double s;               // 在路段上的s值，表示距离起点的距离
        double l;               // 在路段上的l值，表示距离车道中心线的距离
        ObjectType type;        // 对象类型
        XYZArray corner_points; // 对象的坐标
        double height;          // 对象高度

        ObjectOnRoad() = default;
        ~ObjectOnRoad() = default;
    };

    /**
     * @struct StopLine
     * @brief 存储停车线的信息，包括ID、所在道路ID、traffic light group ID、线段的坐标等信息
     * @warning 一个停车线通常与一个红绿灯对应，但有时也会存在多个灯与同一条停车线对应的情况
     */
    struct MAP_API StopLine
    {
        ObjectId id;                  // 停车线ID
        RoadId road_id;               // 所在道路的ID
        TrafficLightGroupId group_id; // 所属交通信号灯组的ID
        std::vector<XYZ> line_xyz;    // 停车线的坐标
        std::vector<SLZ> line_slz;    // 停车线的坐标

        StopLine();
    };

    /**
     * @struct Pole
     * @brief 存储杆子的信息，包括ID、所在车道的SLZ、坐标、高度、姿态等信息
     */
    struct MAP_API Pole
    {
        ObjectId id;   // 杆子ID
        SLZ slz;       // 所在车道的SLZ
        XYZ xyz;       // 杆子的坐标
        double height; // 杆子高度
        double hdg;    // 杆子方向
        double pitch;  // 杆子俯仰角
        double roll;   // 杆子横滚角

        Pole();
    };

    struct MAP_API TrafficLight
    {
        TrafficLightId id;       // 交通信号灯ID
        TrafficLightType type;   // 交通信号灯类型
        RoadId position_road_id; // 交通信号灯所在道路的ID
        double s;                // 交通信号灯在道路上的位置坐标S
        double l;                // 交通信号灯在道路上的位置坐标L
        double zOffset;          // 交通信号灯中心点相对地面的高度
        uint32_t orientation;    // 交通信号灯的方向（1: 正向，-1: 反向）
        double hdg;              // 交通信号灯模型的朝向角（yaw）
        double pitch;            // 交通信号灯模型的前后倾斜角
        double roll;             // 交通信号灯模型的左右翻转角

        RoadId logical_road_id; // 交通信号灯生效的道路ID
        LocalLaneId valid_from; // 交通信号灯生效的起始车道ID
        LocalLaneId valid_to;   // 交通信号灯生效的结束车道ID

        std::vector<ObjectId> reference_list; // 与交通信号灯相关的其他对象的ID列表，比如杆子、停止线等

        TrafficLight() = default;
    };

    struct MAP_API TrafficLightGroup
    {
        std::string group_id;                      // 交通信号灯组的ID
        std::vector<TrafficLightId> group_members; // 交通信号灯组内的成员信号灯ID列表
        ObjectId pole_id;                          // 交通信号灯组对应的杆子的ID

        TrafficLightGroup() = default;
    };

    // ------------------------------------ Route related structures ----------------------------------------------
    /**
     * @struct Route
     * @brief 基本的路线信息，用于存储和传递routing的结果
     */
    struct MAP_API Route
    {
        std::vector<LaneId> lane_id_vec; // routing 所得的路线的Vector版，方便c++端调用
        SLZ begin;                       // routing路线的起始点
        SLZ end;                         // routing路线的目标点
        RouteS length;                   // routing所得路线的总长

        Route();        // 构造函数，需要给予定长的初始化支持
        void arr2vec(); // 将路线转换为vector形式
    };

    /**
     * @struct SampledLine
     * @brief 采样后的中心线信息
     */
    struct MAP_API SampledLine
    {
        std::vector<SLZ> centerline; // 中心线采样点的集合

        SampledLine(); // 默认构造函数
    };

    /**
     * @struct RouteSegment
     * @brief 路线中的一个路段
     */
    struct MAP_API RouteSegment
    {
        LaneId lane_id; // 车道ID
        double start_s; // 路段起始s值
        double end_s;   // 路段结束s值

        RouteSegment() = default; // 默认构造函数
    };

    /**
     * @enum TracePointType
     * @brief 轨迹点的类型枚举（正常点、减速带等）
     */
    typedef enum
    {
        kNormal = 0,    // 正常轨迹点
        kSpeedBump = 1, // 包含减速带的轨迹点
    } TracePointType;

    /**
     * @struct TracePoint
     * @brief 道路中心线上的一个点的信息
     */
    struct MAP_API TracePoint
    {
        double x;                              // 该点的x坐标
        double y;                              // 该点的y坐标
        double z;                              // 该点的z坐标
        LaneId lane_id;                        // 该点所在车道的全局ID
        double s;                              // 该点在车道上的s值
        double l;                              // 该点在车道上的l值
        double hdg;                            // 中心线上该点的全局朝向角
        double curv;                           // 中心线上该点的曲率
        double curv_deriv;                     // 中心线上该点的曲率一阶导数
        SingleLaneRoadMarkType left_roadmark;  // 车道在该点处的左边界标线类型
        SingleLaneRoadMarkType right_roadmark; // 车道在该点处的右边界标线类型
        TracePointType type;                   // 轨迹点的类型
        double left_border;                    // 左边界值
        double right_border;                   // 右边界值

        TracePoint() = default; // 默认构造函数
        TracePoint(double xx, double yy, double zz, LaneId ldld, double ss, double ll, double hh, double cc, double cdcd,
                   SingleLaneRoadMarkType lr, SingleLaneRoadMarkType rr, TracePointType tpt, double left_border_,
                   double right_border_);
    };

    /**
     * @struct SpeedBump
     * @brief 减速带的位置信息
     */
    struct MAP_API SpeedBump
    {
        double s_start; // 减速带起始s值
        double s_end;   // 减速带结束s值
        LaneId lane_id; // 减速带所处车道

        // int valid;       // 是否有效

        SpeedBump(); // 默认构造函数
    };

    /**
     * @struct BusStop
     * @brief 接驳站点的位置信息
     */
    struct MAP_API BusStop
    {
        int id;         // 站点ID
        double s;       // 站点几何中心s值
        double l;       // 站点几何中心l值
        double z;       // 站点几何中心z值
        double length;  // 站点长度
        double width;   // 站点宽度
        double hdg;     // 站点航向角
        double x;       // 站点几何中心x坐标
        double y;       // 站点几何中心y坐标
        LaneId lane_id; // 站点所处车道

        // int valid;       // 是否有效

        BusStop(); // 默认构造函数
    };

    /**
     * @struct StreetLamp
     * @brief 街灯信息结构体定义
     */
    struct MAP_API StreetLamp
    {
        int model;   // 街灯模型
        double hdg;  // 街灯的朝向角度
        XYZ xyz;     // 街灯在地图中的三维坐标
        SLZ slz;     // 街灯在车道上的位置信息
        ObjectId id; // 街灯的全局唯一标识符
    };

    /**
     * @struct Rotation
     * @brief 三维转角结构体定义（绕slz）
     */
    struct MAP_API Rotation
    {
        double roll;  // 绕s方向旋转（车头方向）
        double pitch; // 绕l方向旋转（垂直车头方向）
        double yaw;   // 绕z方向旋转
    };

    /**
     * @struct ParkingSpace
     * @brief 车位信息结构体定义
     */
    struct MAP_API ParkingSpace
    {
        int id;         // 车位的唯一标识符
        double hdg;     // 车位的朝向角度
        RoadId road_id; // 车位相邻道路的标识符
        XYZ corner[4];  // 车位四个角点的三维坐标
    };

    // ------------------------------------ ErrorCode Enumration ----------------------------------------------
    /**
     * @enum ErrorCode
     * @brief Enumerate possible running status of the methods
     */
    enum ErrorCode
    {
        kOK = 0,                 // 方法正常运行
        kFileReadingError = 1,   // 文件路径错误或者xml格式错误
        kLaneLocalIdInvalid = 2, // 车道本地标识符无效
        kMapNotReady = 3,        // 地图未准备好

        kAnchorNotCreated = 4, // 锚点未创建
        kAnchorsNotEnough = 5, // 给出的锚点少于2个
        kAnchorInvalid = 6,    // 锚点无效
        kAnchorRelocated = 7,  // 锚点重定位，当用户所指位置无锚点而自动分配给最近的点时返回
        kAnchorsTooClose = 8,  // 锚点太靠近
        kRoadSInvalid = 9,     // 道路无效
        kRouteSInvalid = 10,   // 路线无效

        kPositionNotInMapBound = 11,    // 位置不在地图边界内
        kUnknownType = 12,              // 未知类型
        kLoadedMapExists = 13,          // 已加载地图存在
        kInvaildPosition = 14,          // 无效位置
        kPositionsTooClose = 15,        // 位置太靠近
        kTooEarlyToChange = 16,         // 更改太早
        kTooLateToChange = 17,          // 更改太晚
        kIlegalLaneChange = 18,         // 非法车道变更
        kPositionNotInAnyJunction = 19, // 位置不在任何交叉口内
        kPositionNotOnRoad = 20,        // 位置不在道路上
        kPositionNotOnLane = 21,        // 位置不在车道上
        kPositionReversed = 22,         // 位置反转
        kDidntGetCoordinate = 23,       // 未获取到坐标
        kSpeedRecordNotFound = 24,      // 未找到速度记录
        kXYZNotOnRoads = 25,            // XYZ坐标不在道路上
        kAnchorListEmpty = 26,          // 锚点列表为空
        kAnchorNoMatches = 27,          // 没有匹配的锚点
        kJunctionIdInvalid = 28,        // 交叉口ID无效
        kRoadIdInvalid = 29,            // 道路ID无效
        kObjectNotFound = 30,           // 未找到对象
        kRoutingNotInitialized = 31,    // 路由未初始化
        kLaneSectionIndexInvalid = 32,  // 车道段索引无效
        kNoTrafficLightFound = 33,      // 未找到红绿灯
        kRouteNotFound = 34,            // 未找到路线
        kNotInMapBound = 35,            // 不在地图边界内
        kSpeedSInvalid = 36,            // 速度无效

        kAnchodIdNotFound = 37,          // 锚点ID未找到
        kAnchorAlreadyExistsInList = 38, // 锚点已存在于列表中
        kMapNotInited = 39,              // 地图未初始化
        kIndexOverflow = 40,             // 索引溢出
        kLaneUidInvalid = 41,            // 车道UID无效
        kJunctionBoundaryInvalid = 42,   // 交叉口边界无效

        kWrongTrafficLightGroup = 43,           // 错误的红绿灯组
        kJunctionBoundaryNotExist = 44,         // 交叉口边界不存在
        kInvalidStartEndPos = 45,               // 无效的起始/结束位置
        kStartEndCurvEql = 46,                  // 起始/结束曲线相等
        kInvalidThresholdValue = 47,            // 阈值无效
        kNoSuccessorExists = 48,                // 不存在后继
        kNoSuccessorJunction = 49,              // 不存在后继交叉口
        kObjectNotCorrect = 50,                 // 对象不正确
        kNoTrafficlightGroupFound = 51,         // 未找到红绿灯组
        kNoStoplineFound = 52,                  // 未找到停车线
        kInCorrectObject = 53,                  // 错误的对象
        kTrafficLightGroupNotFound = 57,        // 未找到红绿灯组
        kStopLineNotFound = 60,                 // 未找到停车线
        kNoSuccessorLaneMatches = 61,           // 没有后继车道匹配
        kNoMapLoaded = 62,                      // 未加载地图
        kSignalNotFound = 63,                   // 未找到信号
        kStopLineHasNoRelatedTrafficLight = 67, // 停车线没有相关的红绿灯
        KNullptr = 68,                          // 空指针
    };

    //------------------------------------------------------ Const Values ----------------------------------------------
    const LaneId EmptyLandId = LaneId();                 // 空的车道标识符
    const XYZ EmptyXYZ = XYZ(DBL_MAX, DBL_MAX, DBL_MAX); // 空的XYZ坐标
    const SLZ EmptySLZ = SLZ();                          // 空的SLZ
    const Anchor EmptyAnchor = Anchor();                 // 空的锚点

    const LaneInfo EmptyLaneInfo = LaneInfo();                // 空的车道信息
    const ObjectInRoute EmptyObjectInRoute = ObjectInRoute(); // 空的路线中的对象
    const Route EmptyRoute = Route();                         // 空的路线
    const SampledLine EmptySampledRoute = SampledLine();      // 空的采样路线

    const LaneIdArray EmptyLaneIdArray = LaneIdArray();                      // 空的车道标识符数组
    const XYZArray EmptyXYZArray = XYZArray();                               // 空的XYZ坐标数组
    const SLZArray EmptySLZArray = SLZArray();                               // 空的SLZ数组
    const AnchorArray EmptyAnchorArray = AnchorArray();                      // 空的锚点数组
    const ObjectInRouteArray EmptyObjectInRouteArray = ObjectInRouteArray(); // 空的路线中的对象数组

    //------------------------------------------------------ LaneChangePolicy type function pointer---------------------
    /**
     * @brief The decleration of the function pointer of the lane change policy
     *
     */
    MAP_API typedef SLZ (*LaneChangePolicy)(const SLZ slz, const LaneId lane_id);

    //-------------------------------------------------------Interface Class --------------------------------------------
    /**
     *  @class Map
     *  @brief Surface class of the route planning oriented OpenDrive HDMap Interface
     *  @details
     */

    class MAP_API Map
    {
    public: // TODO:
        /**
         * @brief Constructor of Map class
         */
        Map();
        /**
         * @brief Destorier of Map class
         */
        ~Map();

        // ------------------------------------ Map data loading/unloading methods ----------------------------------------------
        /**
         * @brief load one map from local file.
         * 		  If there already has map loaded in the Object, replace it.
         *
         *
         * @param file_path: path of map data file
         * @param handle: handle of the loaded map
         * @return ErrorCode： 0：success, otherwise: failed
         */
        ErrorCode load(const char *file_path, int &handle);

        /**
         * @brief delete loaded map from Memory
         * @return ErrorCode： 0：success, otherwise: failed
         */
        ErrorCode unload(int &handle);

        void get_eswn(double &north, double &south, double &east, double &west) const;

        // ------------------------------------ Position / S related methods ----------------------------------------------

        /**
         * @brief transform an UTM-based XYZ coordinate to Road-Reference-Line-based SLZ coordinate
         * @param xyz  XYZ coordinate system
         * @param slz  SLZ coordinate system
         * @param hint  lane_id as hint information for the searching
         * @return  SLZ coordinate
         * 			or the SLZ coordinate of the nearest point on a road
         * 			if not found return SLZ_EMPTY()
         *
         */
        SLZ find_slz(const XYZ &xyz, double radius, const LaneId &hint) const; //

        /**
                 * @brief transform an Road-Reference-Line-based SLZ coordinate to UTM-based XYZ coordinate
                 * @param slz  SLZ coordinate system

                 * @return XYZ coordinate system:
                 * 		   If slz is empty, return XYZ_EMPTY()
                 */
        XYZ xyz(const SLZ &slz) const;

        // ------------------------------------ Anchor related methods ----------------------------------------------
        /**
         * @brief for users to manually create an anchor by specifing the anchor's id and position
         * @param id  proposed anchor's id
         * @param pos  proposed anchor's position in SLZ coordinate system
         * @return the created anchor object, return ANCHOR_EMPTY() if no anchor was created
         */
        Anchor create_anchor(const AnchorId &id, const SLZ &pos) const;

        /**
         * @brief for users to manually add an anchor to an existed anchor list
         * @param anchor anchor to be added
         * @return true:success, false: failed
         */
        static ErrorCode add_anchor(const Anchor &anchor, AnchorArray &anchor_list);

        /**
         * @brief for users to manually delete an anchor from an existed anchor list
         * @param id id of the anchor to be deleted
         * @return true:success, false: failed
         */
        static ErrorCode delete_anchor(const AnchorId &id, AnchorArray &anchor_list);

        /**
         * @brief for users to find the anchor with certain pattern in the anchor list
         * @param pattern pattern of the desired anchor's string id, could be writtern in regular expression
         * @param anchor_list the anchor list to be searched in
         * @param anchor the urged anchor
         * @return true:success, false: failed
         */
        static ErrorCode find_anchor(const char *&pattern, const AnchorArray &anchor_list, AnchorArray &found_anchors);

        // ------------------------------------ Lane related ----------------------------------------------
        /**
         * @brief find the lane with specified id and read information
         * @param id
         * @return lane informations listed in Lane Structure,
         * 		   if Empty return LANE_INFO_EMPTY()
         */
        LaneInfo query_lane_info(const LaneId &id) const;

        /**
         * @brief query the speed limitation at a certain position on lane
         *
         * @param id
         * @param s
         * @return double: speed_limit. If not found return DEL_MAX;
         */
        ErrorCode query_lane_speed_at(const LaneId &id, const RoadS &s, double &speed_limit) const;

        /**
         * @brief query the permitted lane change type at a certain position on lane
         *
         * @param id
         * @param s
         * @param lane_change_type
         * @return LaneChangeType, if not found return kUnknownChangeType
         */
        LaneChangeType query_lane_change_type_at(const LaneId &id, const RoadS &s) const;

        /**
         * @brief query the left and right boundary of the specified lane
         * @param lane_id  the global id of the lane to be queried
         * @param left_boundary  the sample vertices' SLZ coordinate list of the left boundary
         * @param right_boundary  the sample vertices' SLZ coordinate list of the right boundary
         * @return ErrorCode
         */
        ErrorCode query_lane_boundaries(const LaneId &lane_id, double sampling_spacing, SLZArray &left_boundary, SLZArray &right_boundary) const;

        // ErrorCode query_road_mark_between(const LaneId& id_1, const LaneId& id_2, ) const;
        /**
         * @brief calculate the center line in a specified interval on the lane
         * @param id id of the current lane
         * @param s1 start point of the interval
         * @param s2 end point of the interval
         * @param sampling_spacing spacing between sampling points
         * @param centerline the center line in the specified interval on the current lane
         * @return ErrorCode
         */
        ErrorCode calc_lane_center_line(const LaneId &id, const RoadS &s1, const RoadS &s2, double sampling_spacing,
                                        SLZArray &centerline) const;

        ErrorCode get_ROI_lane_line(const XYZ &point, std::vector<XYZ> &roi_points, const double distance = 20.0) const;

        ErrorCode get_ROI_lane_line(const SLZ &point, std::vector<XYZ> &roi_points, const double distance = 20.0) const;

        //  ------------------------------------ Route related methods ----------------------------------------------

        ErrorCode plan_route_shortest(const XYZ &start_point, const std::vector<XYZ> &way_points, const XYZ &end_point, Route &route) const;

        ErrorCode plan_route(const XYZ &start_point, const std::vector<XYZ> &way_points, const XYZ &end_point, Route &route) const;

        /**
         * @brief plan the route from given start/end/way points based on an abstracted Opendrive Map by AStar algorithm.
         *
         * @param start_anchor
         * @param way_point_list
         * @param end_anchor
         * @param route
         * @return ErrorCode
         */
        ErrorCode plan_route(const Anchor &start_anchor,
                             const AnchorArray &way_point_list,
                             const Anchor &end_anchor,
                             Route &route) const;
        /**
         * @brief return a list of sampling points of a user-specified interval on the given route
         *
         * @param route the route which the sample route segment based
         * @param start_route_s the start RouteS of current Sample Route Segment on the route
         * @param length the proposed length of the sample route segment
         * @param default_lane_change_policy for getting the lane change trace without any user input
         * \@param turning_radius the turning radius given by the vehicle state
         * @param sample_route_segment the result route segment sampling point list to be passed out
         * @return ErrorCode: 0 success; otherwise error occurs;
         */
        ErrorCode sample_route(const Route &route,
                               const RouteS &start_route_s,
                               const RouteS &length,
                               SampledLine &sampled_route,
                               LaneChangePolicy lane_change_policy) const;

        // ------------------------------------ Junction related methods ----------------------------------------------

        /**
         * @brief calclate whether the given point is inside a specified junction or not
         *
         * @param junction_id
         * @param pos
         * @return true: the point is inside the junction (including the situation point on the boundary),
         * 		   false: not inside the junction.
         */
        bool is_inside_junction(const JunctionId &junction_id, const SLZ &pos) const;

        /**
         * @brief find the junction's physical outline points if they exist.
         * @param junction_id
         * @param junction_boundary XYZ vertices series in vehicle-heading-wise direction, begin with the nearest
         * 							junction boundary point
         * @return ErrorCode:0 successfully found the jucntion boundary; otherwise error occurs;
         */
        ErrorCode find_junction_boundary(const JunctionId &junction_id, XYZArray &junction_boundary) const;

        // ------------------------------------ Object related methods ----------------------------------------------

        /**
         * @brief return a list of objects sorted by the (forward) distance to the vehicle on the located route
         * @param route
         * @param start_route_s
         * @param length
         * @param object_type_filter
         * @param object_list
         * @return ErrorCode: 0 success; otherwise error occurs;
         */
        ErrorCode find_objects_on_route(const Route &route, const RouteS &start_route_s, const RouteS &length, const ObjectTypeArray &object_type_filter,
                                        ObjectInRouteArray &object_list) const;

        ErrorCode find_objects_on_road(const RoadId &road, std::vector<ObjectOnRoad> &objects) const;
        // ----------------------------------- Case related methods ----------------------------------------------

        //**
        //		* @brief get the heading angle of the lane at the given position on lane
        //		* @param id lane's global id
        //	    * @param pos the given point of the querying
        //	    * @param angle the angle between the lane's heading angle at pos and the east direction
        //		* @return ErrorCode: 0 success; otherwise error occurs;
        // */
        ErrorCode calc_road_heading_angle(const SLZ &pos, double &angle) const;

        ErrorCode calc_road_end_heading_angle(const LaneId &id, double &angle) const;

        ErrorCode query_lane_width_at(const SLZ &pos, double &width) const;

        ErrorCode query_road_width_at(const SLZ &pos, double &width) const;

        ErrorCode query_road_mark_types_between(const SLZ &pos1, const SLZ &pos2, char **&road_mark_types, unsigned int &length_) const;

        ErrorCode find_slz_with_hint(const XYZ &xyz, const LaneId &hint, SLZ &slz) const;
        ErrorCode calc_slz_with_road_id(const XYZ &xyz, const RoadId &road_id, SLZ &slz) const;
        ErrorCode find_slz_global(const XYZ &xyz, SLZ &slz) const;

        ErrorCode find_slzs(const XYZ &xyz, std::vector<SLZ> &slzs) const;

        ErrorCode is_road_in_junction(const RoadId &road_id, JunctionId &junction_id) const;

        // check the boundaries of every lane in the map
        ErrorCode get_all_boundary_points(std::vector<XYZ> &plist);

        // check the distance from a position along the route to the end of the route
        double calc_dist_to_end_anchor(const XYZ &xyz, const std::vector<LaneId> &lane_uids, double start_s, double end_s) const;
        // check if the given point is inside the junction
        bool is_inside_junction(const JunctionId &junction_id, const XYZ &pos) const;

        //
        ErrorCode is_inside_junction(const LaneId &hint, const XYZ &pos, JunctionId &junction_id) const;

        //
        ErrorCode query_lane_linkage(const LaneId &lane_uid, LaneLinkage &lane_linkage) const;

        /*ErrorCode query_lane_neighbor(const LaneId& lane_uid, LaneNeighbor& lane_neighbor) const;*/

        //
        bool is_point_on_road(const RoadId road_id, const XYZ &xyz) const;

        /**
                 * @brief 给定阈值范围内,判断平面上某个点是否处于指定道路的frenet坐标系内
                 * @param road_id 指定道路的id
                 * @param xyz 查询点的绝对坐标
                 * @param threshold 指定点到边界的阈值，在阈值内,表示在道路附近,local_id赋值为离它最近车道的local_id
                 * @param slz 查询点在指定road的frenet坐标系下的相对坐标。
                              当该点未投影到road参考线上时，返回的slz的lane_id的section_idx和local_id均为INT_MAX；
                              当该点投影到road参考线上，但是在道路边界外,并在阈值之外时: 返回的slz的lane_id的local_id为999，section_idx正常；
                              当该点投影到road参考线上，且在道路边界内时,或者在道路边界外并在阈值之内时: 返回的slz正常；
                 * @pre-condition 地图已成功加载，road_id 存在于地图上，distance>0
                 * @post-condition 无
                 * @return 若点能够投影到道路参考线上则返回true，否则返回false
                */
        bool is_point_in_road_range(const RoadId &road_id, const XYZ &xyz, double threshold, SLZ &slz) const;

        ErrorCode query_lane_boundaries(const LaneId &lane_id, const RoadS &s1, const RoadS &s2, double sampling_spacing, SLZArray &left_boundary, SLZArray &right_boundary) const;

        //
        ErrorCode calc_lane_center_line_curv(const LaneId &id, const RoadS &s1, const RoadS &s2, double sampling_spacing, std::vector<TracePoint> &centerline) const;

        void get_all_street_lamps(std::vector<StreetLamp> &street_lamps) const;
        void get_street_lamps_by_road(const RoadId road_id, std::vector<StreetLamp> &street_lamps) const;

        ErrorCode get_all_speed_bump(std::vector<SpeedBump> &speed_bumps) const;

        ErrorCode get_all_bus_stop(std::vector<BusStop> &bus_stops) const;

        bool get_bus_stop_by_lane(const LaneId &id, std::vector<BusStop> &bus_stops) const;

        bool get_bus_stop_by_road(const RoadId road_id, std::vector<BusStop> &bus_stops) const;

        ErrorCode get_all_parking_space(std::vector<ParkingSpace> &parking_spaces) const;
        bool get_parking_space_by_road(const RoadId road_id, std::vector<ParkingSpace> &parking_spaces) const;
        bool is_point_in_parking_space(const XYZ &xyz) const;
        bool is_point_in_parking_space(const XYZ &xyz, const ParkingSpace &parking_space) const;
        bool get_parking_space_by_xyz(const XYZ &xyz, ParkingSpace &parking_space) const;

        ErrorCode get_all_laneids(std::vector<LaneId> &all_laneids) const;

        ErrorCode get_rotation_by_slz(const SLZ &slz, Rotation &rotation) const;
        ErrorCode get_rotation_by_xyz(const XYZ &point, Rotation &rotation) const;
        ErrorCode get_rotation_by_xyz_with_hint(const LaneId &hint, const XYZ &point, Rotation &rotation) const;

        ErrorCode get_z_by_sl(SLZ &slz) const;

        ErrorCode get_z_by_xy_with_road_id(const RoadId &road_id, XYZ &xyz) const;

        ErrorCode get_z_by_xy_with_last_z(double last_z, XYZ &xyz) const;

        double get_road_length(const RoadId &road_id) const;

        //----------------------------------------------MS8 Traffic Light
        /**
         * @brief 给出Road Id，查询道路上所有红绿灯的Id
         * @param road_id
         * @param traffic_lights_ids 传出的红绿灯id vecor
         * @return ErrorCode: 0 success; otherwise error occurs;
         */
        ErrorCode query_traffic_lights_in_road(const RoadId &road_id, std::vector<ObjectId> &traffic_lights_ids) const;

        /**
         * @brief 查询地图上所有红绿灯Id
         * @param road_id
         * @param traffic_lights_ids 传出的红绿灯id vecor
         * @return ErrorCode: 0 success; otherwise error occurs;
         */
        ErrorCode query_all_traffic_light_groups(std::vector<TrafficLightGroup> &traffic_light_groups_id) const;

        /**
         * @brief 输入一个SLZ位置，给出该位置所处车道的转向类型。
         * @param slz
         * @param turn_type 传出转向类型，如果当前车道后继不是junction，则返回LaneTurnType::k;目前不考虑kUnkonwnTurnType，只有ErrorCode 非 kOK 时才有 turn_type = kUnkonwnTurnType。
         * @return ErrorCode: 0 nothing wrong; otherwise error occurs;
         */
        ErrorCode query_lane_turn_type(const SLZ &slz, LaneTurnTypeOrigin &turn_type) const;

        ErrorCode query_lane_turn_type(const SLZ &slz, LaneTurnType &turn_type) const;

        /**
            * @brief 输入一个SLZ位置，给出该位置前方第一个junction的Id。
                    1. 查到junction。此时若距离大于distance则认为前方无junction，返回空JunctionId = -1；否则返回对应junctionId
                    2. 后继未查到junction，认为前方无junction，返回空JunctionId = -1；
            * @param slz
            * @param distance 前方查询距离
            * @param junc_id 传出的junction id
            * @return ErrorCode: 0 success; otherwise error occurs;
            */
        ErrorCode query_forward_junction_by_slz(const SLZ &slz, double distance, JunctionId &junc_id) const;

        /**
         * @brief 输入一个SLZ位置，给出该位置前方的最近一个行车红绿灯组
         * @param slz
         * @param distance 前方查询距离
         * @param type 指定查询的红绿灯类型（行车，行人，自行车）
         * @param traffic_light_group 传出的红绿灯组，若前方distance距离内无红绿灯组，返回EmptyTrafficLightGroup
         * @return ErrorCode: 0 nothing wrong; otherwise error occurs;
         */
        ErrorCode query_forward_traffic_light_group_by_slz(const SLZ &slz, double distance, TrafficLightType type, TrafficLightGroup &traffic_light_group) const;

        /**
         * @brief 输入一个SLZ位置，给出该位置前方第一个停止线的Id。若无,返回EmptyObjectId（空字符串）
         * @param slz
         * @param distance 前方查询距离
         * @param stop_line_id 传出的stopline id
         * @return ErrorCode: 0 nothing wrong; otherwise error occurs;
         */
        ErrorCode query_forward_stopline_by_slz(const SLZ &slz, double distance, ObjectId &stop_line_id) const;

        /**
         * @brief 通过id查询红绿灯对象
         * @param traffic_light_id
         * @param traffic_light 传出的红绿灯对象，若无该id对应的红绿灯，返回EmptyTrafficLight{id = "-1"}
         * @return ErrorCode: 0 nothing wrong; otherwise error occurs;
         */
        ErrorCode get_traffic_light_info_by_id(const TrafficLightId &traffic_light_id, TrafficLight &traffic_light) const;

        /**
         * @brief 通过id查询停止线对象
         * @param stopline_id
         * @param stopline 传出的停止线对象，若无该id对应的停止线，返回EmptyStopLine{id = "-1"}
         * @return ErrorCode: 0 nothing wrong; otherwise error occurs;
         */
        ErrorCode get_stopline_info_by_id(const ObjectId &stopline_id, StopLine &stopline) const;

        /**
         * @brief 通过id查询Pole对象
         * @param pole_id
         * @param pole 传出的灯杆对象，若无该id对应的灯杆，返回EmptyPole{id = "-1"}
         * @return ErrorCode: 0 nothing wrong; otherwise error occurs;
         */
        ErrorCode get_pole_info_by_id(const ObjectId &pole_id, Pole &pole) const;

        /**
         * @brief 通过红绿灯组id获取红绿灯组对象
         * @param traffic_light_group_id
         * @return TrafficLightGroup， 若未查询到，则返回EmptyTrafficLightGroup
         */
        ErrorCode get_traffic_light_group_info_by_id(const TrafficLightGroupId &traffic_light_group_id, TrafficLightGroup &tlg) const;

        /**
         * @brief 通过红绿灯id查询其所处的组的id
         * @param traffic_light_id
         * @param traffic_light_group_id
         * @return ErrorCode: 0 nothing wrong; otherwise error occurs;
         */
        ErrorCode query_traffic_light_group_id_by_member(const TrafficLightId &traffic_light_id, TrafficLightGroupId &traffic_light_group_id) const;

        /**
         * @brief 输入两个车道，源车道与目标车道，给出从源车道到目标车道的转向类型。
         * @param origin_lane_id 源（起始）车道
         * @param target_lane_id 目标车道
         * @param turn_type 传出转向类型，只有当目标车道是源车道的后继车道时，才返回LaneTurnType::k，否则以ErrorCode非0表示kUnkonwnTurnType。
         * @return ErrorCode: 0 nothing wrong; otherwise error occurs;
         */
        /**
         * @brief 输入两个车道，源车道与目标车道，给出从源车道到目标车道的转向类型。
         * @param origin_lane_id 源（起始）车道
         * @param target_lane_id 目标车道
         * @param turn_type 传出转向类型，只有当目标车道是源车道的后继车道时，才返回LaneTurnType::k，否则以ErrorCode非0表示kUnkonwnTurnType。
         * @return ErrorCode: 0 nothing wrong; otherwise error occurs;
         */
        ErrorCode query_lane2lane_turn_type(const LaneId &origin_lane_id, const LaneId &target_lane_id, LaneTurnTypeOrigin &turn_type) const;

        /**
         * @brief 通过红绿灯组id查询停止线对象 id
         * @param traffic_light_group_id
         * @param stopline 传出的停止线对象，若无该id对应的停止线，返回EmptyStopLine{id = "-1"}
         * @return ErrorCode: 0 nothing wrong; otherwise error occurs;
         */
        ErrorCode query_stopline_by_traffic_light_group_id(const TrafficLightGroupId &traffic_light_group_id, ObjectId &stopline_id) const;

        //----------------------------------------------After ms8
        /*struct ParkingSpace {
            ObjectId id;
            std::vector<XYZ> outline;
            double hdg;
        };*/

        ErrorCode query_all_roads_in_junction(const JunctionId &junction_id, std::vector<RoadId> &road_id_vec) const;

        // ErrorCode query_parking_info(const RoadId& road_id, ParkingSpace& parking_space)  const;

        //----------------------------------------------Auxiliary
        // bool is_road_id_valid(RoadId road_id);
        bool is_lane_id_valid(LaneId lane_id) const;
        bool is_slz_valid(SLZ slz) const;
        /**
         * @brief: Query the lane's roadmark on both side at specified s positions
         * @param lane_id: The input lane global id
         * @param s: The querying position s
         * @param road_mark_pair: (left roadmark, right roadmark), roadmark value are written in string like  					 		 * 						  "{$type} {$color}"
         * @return kOK:success, otherwise: failed
         */
        ErrorCode query_lane_road_mark(const LaneId &lane_id, double s, std::pair<std::string, std::string> &road_mark_pair) const;

        ErrorCode calc_spiral_points(const TracePoint &start_point, const TracePoint &end_point, double length, double sampling_spacing, std::vector<TracePoint> &result) const;

        bool is_point_around_road(const RoadId &road_id, const XYZ &xyz, double &distance, SLZ &slz) const;

    private:
        class MapImpl;
        MapImpl *map_impl_ap_;

        /**
         * @brief Copy Operation (just decleared, not implemented)
         */
        Map(const Map &);
        /**
         * @brief Copy Operator (just decleared, not implemented)
         */
        Map &operator=(const Map &);
    };
} // namespace zjlmap

#endif // !MAP_H_
