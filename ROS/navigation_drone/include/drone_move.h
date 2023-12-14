// 矩阵运算库
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Core>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

// 无人机相关的库
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/BatteryStatus.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

// ROS相关的库
#include <sensor_msgs/BatteryState.h>
#include <ros/ros.h>
#include <iostream>
#include <vector>

// 位姿相关库
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

// 无人机状态
#define _INIT_ 0
#define _ARMED_ 1
#define _LAND_ 2
#define _HOVER_ 3
#define _MOVING_ 4
#define _FAILSAFE_ 5 // 电量过低或者失控

// 路线状态码
#define _MAP_IS_NULL_ 0
#define _SIZE_IS_LESS_THAN_ZERO_ 1
#define _BOUND_IS_NOT_INITIALIZED_ 2
#define _RESOLUTION_IS_LESS_THAN_ZERO_ 3
#define _EXPECTED_ROUTE_IS_EMPTY_ 4
#define _EXPECTED_ROUTE_IS_BLOCKED_ 5
#define _EXPECTED_ROUTE_IS_CLEAR_ 6
#define _DRONE_AROUND_IS_NOT_CLEAR_ 7

#include "rrt.h"
#include "pid.h"

// 定义无人机状态
class DroneStatus{
private:  
    uint8_t isObs = -1;
    int GLX_SIZE = -1, GLY_SIZE = -1, GLZ_SIZE = -1;       // 地图大小
    double gl_xl = -1e9, gl_yl = -1e9, gl_zl = -1e9;            // lower bound
    double detect_range = 2.0;                                                // 探测范围
    double resolution = -1, inv_resolution = -1;               // 地图分辨率
    uint8_t * map = NULL;                                                         // 地图
    bool battery_is_set = false;                   
    double range_threshold = 0.05;                                         // 0.05 meter 
    double expected_velocity = 0.1;                                       // 允许速度 0.1m/s
    std::vector<Eigen::Vector3d> theRoute;                         // 设置的新路线
    mavros_msgs::PositionTarget expected_posture;    // 期望位置姿态
    geometry_msgs::PoseStamped currect_posture;     //  当前真实的位置姿态
    mavros_msgs::State current_state;                        // 当前状态
    sensor_msgs::BatteryState current_battery; // 当前电量
    uint8_t status = _INIT_;                                               // 初始化中
    mavros_msgs::SetMode offb_set_mode;           // 设置为offboard
    mavros_msgs::CommandBool arm_cmd;          // 解锁标志位
    bool need_rc = false;                                                    // 是否需要遥控
public:
    DroneStatus();
    void setExpectedVelocity(double new_expected_velocity);
    void setDetectRange(double new_detect_range);
    void setCurrectPosture(geometry_msgs::PoseStamped Posture);
    void setRangeThreshold(double new_threshold);
    void setNewRoute(std::vector<Eigen::Vector3d> & newRoute);
    void setState(mavros_msgs::State state);
    void setBattery(sensor_msgs::BatteryState battery);
    void setStatus(uint8_t new_status);
    void setMinAzimuth(double minAzimuth);
    void setArm();
    // 设置地图以及地图的大小
    void setIsObs(uint8_t obs);
    void setMap(uint8_t * data);
    void setMapSizeX(int size_x);
    void setMapSizeY(int size_y);
    void setMapSizeZ(int size_z);
    void setResolution(double _res_);
    void setMapGLx(double _gl_xl);
    void setMapGLy(double _gl_yl);
    void setMapGLz(double _gl_zl);
    bool setMode();
    void checkMode();
    void resetExpectedPosture();
    void update();
    // 检测障碍物 (Azimuth是上下15度 Elevation是上下10度)
    uint8_t detect_the_obscale();
    // 执行theRoute 中的每个路径点（同时，检查每个路径点上的障碍物）
    void execute_the_route();
};

void getStateCallBack(const mavros_msgs::State::ConstPtr & state);
void getBatteryCallBack(const sensor_msgs::BatteryState::ConstPtr & battery);