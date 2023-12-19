#include "callback_functions.h"
#include "drone_move.h"
#include "rrt.h"

extern double _resolution, _inv_resolution, _cloud_margin;
extern double _x_size, _y_size, _z_size;    
extern double drone_fly_height ; // 默认飞行高度
extern bool new_att_world, new_att_drone, new_drone_world;

extern Eigen::Vector3d _start_pt;
extern Eigen::Vector3d _map_lower, _map_upper;
extern int _max_x_id, _max_y_id, _max_z_id;
extern std::string map_link, drone_link, camera_link;

ros::Subscriber _map_sub, _pts_sub, _odom_sub;
extern ros::Publisher  _grid_map_vis_pub, _RRTstar_path_vis_pub, _b_spline_path_vis_pub;

extern tf::StampedTransform body_to_map, depth_to_drone, depth_to_map;
// RRT map main class
extern RRTstarPreparatory * _RRTstar_preparatory ;
// 观测到的点云
extern pcl::PointCloud<pcl::PointXYZ> cloud_vis;
// 一些测试符号
extern bool debug_pc, debug_path, send_map;
// 是否显示姿态
bool posture_info = false;
// 设置近距离阈值和远距离阈值
extern double close_thres, far_thres; 
// 无人机相关的消息发布与服务发布
extern ros::Subscriber state_sub, battery_sub;
extern ros::Publisher local_pos_pub;
extern ros::ServiceClient arming_client, set_mode_client;
extern DroneStatus * droneStatus;
// 阈值体积
extern int expand_range;
// 低电压
extern double low_battery;
// 无人机的最大允许速度
extern double velocity_drone;
extern double acc_drone;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rrt_demo");
    ros::NodeHandle nh;

    ROS_INFO("[node] RRT_NODE Start");

    _odom_sub = nh.subscribe("/mavros/local_position/pose", 1, rcvCurrentPoseCallback);
    _pts_sub  = nh.subscribe( "/move_base_simple/goal", 1, rcvWaypointsCallback );
    _map_sub  = nh.subscribe( "/voxel_cloud", 1, rcvPointCloudCallBack );

    state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, getStateCallBack);
    battery_sub = nh.subscribe<sensor_msgs::BatteryState>("/mavros/battery", 10, getBatteryCallBack);
    local_pos_pub = nh.advertise<mavros_msgs::PositionTarget> ("/mavros/setpoint_raw/local", 10);
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    // 发布grid map
    // 发布 RRT 路径
    _grid_map_vis_pub             = nh.advertise<sensor_msgs::PointCloud2>("/grid_map_vis", 1);
    _RRTstar_path_vis_pub     = nh.advertise<visualization_msgs::Marker>("/rrt_star_path_vis", 1);
    _b_spline_path_vis_pub   = nh.advertise<visualization_msgs::Marker>("/b_spline_path_vis", 1);

    nh.param("map/cloud_margin",  _cloud_margin, 0.0);
    nh.param("map/resolution",    _resolution,   0.05);
    
    nh.param("map/x_size", _x_size, 20.0);
    nh.param("map/y_size", _y_size, 20.0);
    nh.param("map/z_size", _z_size, 10.0 );
    
    nh.param("planning/start_x",  _start_pt(0),  0.0);
    nh.param("planning/start_y",  _start_pt(1),  0.0);
    nh.param("planning/start_z",  _start_pt(2),  0.0);

    nh.param<std::string>("map_link", map_link, "/map");
    nh.param<std::string>("drone_link", drone_link, "/base_link");
    nh.param<std::string>("camera_link", camera_link, "/camera_link");
    nh.param<double>("drone_flying_height", drone_fly_height, 0.75);
    nh.param<double>("close_threshold", close_thres, 1.0);
    nh.param<double>("far_tbreshold", far_thres, 6.0);

    nh.param<int>("expand_range", expand_range, 4); // 正负4
    nh.param<bool>("debug_pc", debug_pc, false);
    nh.param<bool>("debug_path", debug_path, false);
    nh.param<bool>("send_map", send_map, true);
    nh.param<bool>("print_posture", posture_info, false);
    nh.param<double>("low_battery", low_battery, 3.5);
    nh.param<double>("velocity_drone", velocity_drone, 0.5); // 最大允许速度
    nh.param<double>("acc_drone", acc_drone, 5.0); // 最大允许加速度

    // 偏航角PID
    double yaw_p = 0.08025, yaw_i = 0.06015, yaw_d = 0.02725;
    // 设置PID
    nh.param<double>("yaw_p", yaw_p, 1.754225);
    nh.param<double>("yaw_i", yaw_i, 0.09125); 
    nh.param<double>("yaw_d", yaw_d, 0.03855); 
    _map_lower << - _x_size/2.0, - _y_size/2.0,     0.0;
    _map_upper << + _x_size/2.0, + _y_size/2.0,  +_z_size ;
    _inv_resolution = 1.0 / _resolution;
    
    _max_x_id = (int)(_x_size * _inv_resolution);
    _max_y_id = (int)(_y_size * _inv_resolution);
    _max_z_id = (int)(_z_size * _inv_resolution);

    ROS_INFO("[node] Map size is %d , %d, %d.", _max_x_id, _max_y_id, _max_z_id);

    _RRTstar_preparatory  = new RRTstarPreparatory();
    _RRTstar_preparatory  -> initGridMap(_resolution, _map_lower, _map_upper, _max_x_id, _max_y_id, _max_z_id);
    
    // 设置无人机相对于地图的参数
    droneStatus->setMap(_RRTstar_preparatory->getMap());
    droneStatus->setMapSizeX(_RRTstar_preparatory->getMapSizeX());
    droneStatus->setMapSizeY(_RRTstar_preparatory->getMapSizeY());
    droneStatus->setMapSizeZ(_RRTstar_preparatory->getMapSizeZ());
    droneStatus->setMapGLx(_RRTstar_preparatory->getMapLowerBoundX());
    droneStatus->setMapGLy(_RRTstar_preparatory->getMapLowerBoundY());
    droneStatus->setMapGLz(_RRTstar_preparatory->getMapLowerBoundZ());
    droneStatus->setIsObs(_RRTstar_preparatory->getObsNum());
    droneStatus->setResolution(_resolution);
    droneStatus->setYawPID(yaw_p, yaw_i, yaw_d);

    ros::Rate rate(10);

    tf::TransformListener listener;
    if (listener.waitForTransform(drone_link, camera_link, ros::Time(0), ros::Duration(10.0))){
        ROS_INFO("[node] Transform to Drone is OK");
    }else{
        ROS_ERROR("[node] Transform to Drone is Error");
        exit(-1);
    }
    if (listener.waitForTransform(map_link, drone_link, ros::Time(0), ros::Duration(10.0))){
        ROS_INFO("[node] Transform to Map is OK");
    }else {
        ROS_ERROR("[node] Transform to Map is Error");
        exit(-1);
    }
    if (listener.waitForTransform(map_link, camera_link, ros::Time(0), ros::Duration(10.0))){
        ROS_INFO("[node] Transform from map to Camera is OK");
    }else {
        ROS_ERROR("[node] Transform to Map is Error");
        exit(-1);
    }

    bool status = ros::ok();
    while(status) 
    {
        ros::spinOnce();    
        try {
             listener.lookupTransform(map_link, drone_link, ros::Time(0), body_to_map);
             new_att_world = true;
        }catch(tf::TransformException & ex) {
            ROS_ERROR("%s",ex.what());
             ros::Duration(1.0).sleep();
            continue;
        }
        try {
             listener.lookupTransform(drone_link, camera_link, ros::Time(0), depth_to_drone);
             new_att_drone = true;
        }catch(tf::TransformException & ex) {
            ROS_ERROR("%s",ex.what());
             ros::Duration(1.0).sleep();
            continue;
        }
        try {
             listener.lookupTransform(map_link, camera_link, ros::Time(0), depth_to_map);
             new_drone_world = true;
        }catch(tf::TransformException & ex) {
            ROS_ERROR("%s",ex.what());
             ros::Duration(1.0).sleep();
            continue;
        }

        if (posture_info){
            cout << "transition of drone from map: X " << body_to_map.getOrigin().getX() 
            << " Y " << body_to_map.getOrigin().getY() 
            << " Z " << body_to_map.getOrigin().getZ()
            << endl;

            cout << "transition of camera from map: X " << depth_to_map.getOrigin().getX() 
            << " Y " << depth_to_map.getOrigin().getY() 
            << " Z " << depth_to_map.getOrigin().getZ()
            << endl;

            cout << "rotation of drone from map: X " << body_to_map.getRotation().getX() 
            << " Y " << body_to_map.getRotation().getY() 
            << " Z " << body_to_map.getRotation().getZ()
            << " W " << body_to_map.getRotation().getW()
            << endl;

            cout << "rotation of camera from map: X " << depth_to_map.getRotation().getX() 
            << " Y " << depth_to_map.getRotation().getY() 
            << " Z " << depth_to_map.getRotation().getZ()
            << " W " << depth_to_map.getRotation().getW()
            << endl;
        }

        droneStatus->update();
        status = ros::ok();
        rate.sleep();
    }
    delete _RRTstar_preparatory;
    delete droneStatus;
    return 0;
}
