#include "callback_functions.h"
#include "rrt.h"
// 加载b样条曲线
#include "bspline_generate.h"
#include "drone_move.h"

double _resolution, _inv_resolution, _cloud_margin;
double _x_size, _y_size, _z_size;    
double close_thres, far_thres; // 近距离阈值 以及 远距离阈值
double drone_fly_height = 1.0; // 默认飞行高度
bool new_att_drone = false, new_att_world = false, new_drone_world=false; // 检查无人机的姿态变换
int expand_range = 5; // 检测体积
// 测试点云
bool debug_pc = false;
// 测试路径
bool debug_path = false;
// 发送单帧点云还是整个地图
bool send_map = false;

Eigen::Vector3d _start_pt;
Eigen::Vector3d _map_lower, _map_upper;
int _max_x_id, _max_y_id, _max_z_id;
std::string map_link, drone_link, camera_link;

tf::StampedTransform body_to_map, depth_to_drone, depth_to_map;
// RRT map main class
RRTstarPreparatory * _RRTstar_preparatory = NULL;
// 观测到的点云
pcl::PointCloud<pcl::PointXYZ> cloud_vis;
// 发布路径和观测到的点云
ros::Publisher  _grid_map_vis_pub, _RRTstar_path_vis_pub;
// 发布生成的B样条曲线
ros::Publisher _b_spline_path_vis_pub; 
// 设置初始的姿态
geometry_msgs::PoseStamped init_pos, subsequence_pos;
bool init_pos_set = false;
// 外部无人机状态
extern DroneStatus * droneStatus;

class ValidityChecker : public ob::StateValidityChecker
{
public:
    ValidityChecker(const ob::SpaceInformationPtr& si) :ob::StateValidityChecker(si) {}
    //返回给定状态的位置是否与圆形障碍物重叠
    bool isValid(const ob::State* state) const
    {   
        //在本例中使用的是RealVectorStateSpace，因此需要将状态转换为特定类型。
        const ob::RealVectorStateSpace::StateType* state3D =state->as<ob::RealVectorStateSpace::StateType>();
        auto x=(*state3D)[0];
        auto y=(*state3D)[1];
        auto z=(*state3D)[2];

        return _RRTstar_preparatory->isObsFree(x, y, z, expand_range, expand_range, expand_range);
    }
};

void rcvCurrentPoseCallback(const geometry_msgs::PoseStamped & pos){
    _start_pt(0) = pos.pose.position.x;
    _start_pt(1) = pos.pose.position.y;
    _start_pt(2) = pos.pose.position.z;
    droneStatus->setCurrectPosture(pos);
}

void rcvWaypointsCallback(const geometry_msgs::PoseStamped & wp)
{     
    ROS_INFO("[node] receive the planning target");
    ROS_INFO("[node] target location: %f, %f, %f", 
	wp.pose.position.x,
	wp.pose.position.y,
	drone_fly_height);

     ROS_INFO("[node] start location: %f, %f, %f", 
	_start_pt(0),
	_start_pt(1),
	_start_pt(2));

    //if( wp.pose.position.z <= 0.0)
    //    return;

    if(abs(wp.pose.position.x) <= 0.5 && abs(wp.pose.position.y) <= 0.5)
        return;

    Vector3d target_pt;
    target_pt << wp.pose.position.x,
                 wp.pose.position.y, drone_fly_height;
                 //wp.pose.position.z;

    std::thread path_finding = std::thread(pathFinding, _start_pt, target_pt);
    path_finding.detach();
    // pathFinding(_start_pt, target_pt); 
}

// PointCloud2 -> pcl point clouds
void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 & pointcloud_map)
{   
    double pi = 3.1415926535;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    sensor_msgs::PointCloud2 map_vis, map_transfrom;
    
    if ( !new_att_drone || !new_att_world || !new_drone_world ) return;

    // 显示点云
    cloud_vis.clear();
    pcl::PointXYZ pt;
    Eigen::Matrix4f body_transform, camera_transform, map_camera_transform;  
    pcl_ros::transformAsMatrix (depth_to_map, map_camera_transform);
    pcl_ros::transformPointCloud (map_camera_transform, pointcloud_map, map_transfrom);
    pcl::fromROSMsg(map_transfrom, cloud);
    if( (int)cloud.points.size() == 0 ) return;

    ofstream file;

    // 查看点云坐标系是否正确
    if (debug_pc)
        file.open("test_point_clouds.txt");

    for (int idx = 0; idx < (int)cloud.points.size(); idx++)
    {    
        pt = cloud.points[idx];        
        Eigen::MatrixXd new_loc(1, 3);
        new_loc  << pt.x, pt.y, pt.z;

        if (isnan(pt.x) || isnan(pt.y) || isnan(pt.z)) {
            continue;
        }

        if ((new_loc - _start_pt).norm() > far_thres || (new_loc - _start_pt).norm() < close_thres){
            // 太远的也不需要
            continue;
        }

        // set obstalces into grid map for path planning
        _RRTstar_preparatory->increaseObs(new_loc(0, 0), new_loc(0, 1), new_loc(0, 2));
        if (!send_map){
            pcl::PointXYZ temp(new_loc(0, 0), new_loc(0, 1), new_loc(0, 2));
            cloud_vis.points.push_back(temp);
        }
        if (debug_pc)
            file << new_loc(0, 0) << " " << new_loc(0, 1) << " " << new_loc(0, 2) << endl;
    }

    if (debug_pc) {
        file.close();
        exit(0);
    }

    // 更新地图并发送
    if (send_map){
        int MAP_X = _RRTstar_preparatory->getMapSizeX(),
        MAP_Y = _RRTstar_preparatory->getMapSizeY(),
        MAP_Z = _RRTstar_preparatory->getMapSizeZ();
        const uint8_t * map_ = _RRTstar_preparatory->getMap();

        for (int ii = 0; ii < MAP_X; ii++) {
            for (int jj = 0; jj < MAP_Y; jj++) {
                for (int kk = 0; kk < MAP_Z; kk++) {
                    if (map_[ii * MAP_Y * MAP_Z + jj * MAP_Z + kk] == _RRTstar_preparatory->getObsNum()) {
                        Eigen::Vector3d coor = _RRTstar_preparatory->gridIndex2coord(Eigen::Vector3i(ii, jj, kk));
                        pcl::PointXYZ temp(coor(0), coor(1), coor(2));
                        cloud_vis.points.push_back(temp);
                    }else {
                        _RRTstar_preparatory->updateObs(ii, jj, kk);
                    }
                }
            }
        }
    }else {
        int MAP_X = _RRTstar_preparatory->getMapSizeX(),
        MAP_Y = _RRTstar_preparatory->getMapSizeY(),
        MAP_Z = _RRTstar_preparatory->getMapSizeZ();

        for (int ii = 0; ii < MAP_X; ii++) {
            for (int jj = 0; jj < MAP_Y; jj++) {
                for (int kk = 0; kk < MAP_Z; kk++) {
                    _RRTstar_preparatory->updateObs(ii, jj, kk);
                }
            }
        }
    }

    cloud_vis.width    = cloud_vis.points.size();
    cloud_vis.height   = 1;
    cloud_vis.is_dense = true;

    pcl::toROSMsg(cloud_vis, map_vis);

    map_vis.header.frame_id = map_link.c_str();
    map_vis.header.stamp = ros::Time::now();
    _grid_map_vis_pub.publish(map_vis);
}

//返回表示优化目标的结构，以用于优化运动规划。该方法返回一个目标，该目标试图最小化计算路径在配置空间中的长度。
ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si)
{
    return ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si));
}

ob::OptimizationObjectivePtr getThresholdPathLengthObj(const ob::SpaceInformationPtr& si)
{
    ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
    obj->setCostThreshold(ob::Cost(1.51));
    return obj;
}

void pathFinding(const Vector3d start_pt, const Vector3d target_pt)
{
     //创建正在规划的机器人状态空间。
        ob::StateSpacePtr space(new ob::RealVectorStateSpace(3));

        //将空间的边界设置为[0,1]。
        ob::RealVectorBounds bounds(3);
        bounds.setLow(0, - _x_size * 0.5);
        bounds.setLow(1, - _y_size * 0.5);
        bounds.setLow(2, - _z_size * 0.5);

        bounds.setHigh(0, + _x_size * 0.5);
        bounds.setHigh(1, + _y_size * 0.5);
        bounds.setHigh(2, + _z_size * 0.5);
        // -x_max / 2 --- +x_max / 2
        // -y_max / 2 --- +y_max / 2
        // 0          --- z_max
        space->as<ob::RealVectorStateSpace>()->setBounds(bounds);

        //为此状态空间构造一个空间信息实例
        ob::SpaceInformationPtr si(new ob::SpaceInformation(space));
        //设置用于检查空间中哪些状态有效的对象
        si->setStateValidityChecker(ob::StateValidityCheckerPtr(new ValidityChecker(si)));
        si->setup();

        //设置机器人的开始状态
        ob::ScopedState<> start(space);
        start[0]=(&start_pt)->operator[](0);
        start[1]=(&start_pt)->operator[](1);
        start[2]=(&start_pt)->operator[](2);

        //设置机器人的目标状态
        ob::ScopedState<> goal(space);
        goal[0]=(&target_pt)->operator[](0);
        goal[1]=(&target_pt)->operator[](1);
        goal[2]=(&target_pt)->operator[](2);

        //创建问题实例，将变量定义为pdef
        auto pdef(std::make_shared<ob::ProblemDefinition>(si));

        //设置开始和目标状态
        pdef->setStartAndGoalStates(start, goal);

        //设置优化目标，可以选择的选项在前面已经定义：getPathLengthObjective（）和getThresholdPathLengthObj（）
        pdef->setOptimizationObjective(getPathLengthObjective(si));
        
        //使用RRTstar算法构建我们的优化计划器，将Variable定义为optimizingPlanner
        ob::PlannerPtr optimizingPlanner(new og::RRTstar(si));

        //为规划者提供解决问题的实例
        optimizingPlanner->setProblemDefinition(pdef);
        optimizingPlanner->setup();

        // 尝试在两秒钟内解决规划问题
        ob::PlannerStatus solved = optimizingPlanner->solve(2.0);

        if (solved)
        {
            ROS_INFO("[node] Solved Path");
            //从问题定义中获取目标表示（与目标状态不同），并查询找到的路径
            og::PathGeometric* path = pdef->getSolutionPath()->as<og::PathGeometric>();
            vector<Vector3d> path_points;

            for (size_t path_idx = 0; path_idx < path->getStateCount (); path_idx++)
            {
                const ob::RealVectorStateSpace::StateType *state = path->getState(path_idx)->as<ob::RealVectorStateSpace::StateType>(); 
                //将找到的路径从路径转换为rviz显示的路径点
                auto x = (*state)[0];
                auto y = (*state)[1];
                auto z = (*state)[2];
                Vector3d temp_mat(x,y,z);
                path_points.push_back(temp_mat); 
            }
            visRRTstarPath(path_points);       
            visRRTstarPathBSpline(path_points);
        }else {
            ROS_WARN("[node] No Path found");
        }
}

void visRRTstarPath(vector<Vector3d> nodes )
{
    ROS_INFO("[node] Using RRT node");
    //点是绿色，线是绿色
    visualization_msgs::Marker Points, Line; 
    Points.header.frame_id = "map";
    Points.header.stamp    =  ros::Time::now();
    Points.ns              =  "demo_node/RRTstarPath";
    Points.action          =  visualization_msgs::Marker::ADD;
    Points.pose.orientation.w = Line.pose.orientation.w = 1.0;
    Points.id = 0;
    Points.type = visualization_msgs::Marker::POINTS;
    Points.scale.x = _resolution/2; 
    Points.scale.y = _resolution/2;
    Points.color.g = 1.0f;
    Points.color.a = 1.0;

    Line.header.frame_id = "map";
    Line.header.stamp    = ros::Time::now();
    Line.ns              = "demo_node/RRTstarPath";
    Line.action          = visualization_msgs::Marker::ADD;
    Line.id   = 1;
    Line.type   = visualization_msgs::Marker::LINE_STRIP;
    Line.scale.x   = _resolution/2;
    Line.color.b   = 1.0;
    Line.color.a   = 1.0;

    geometry_msgs::Point pt;
    vector<Vector3d> nodes2 = interpolation_to_RRT(nodes);
    ROS_INFO("[node] interpolate node complete");

    if (debug_path){
        ofstream file1, file2;
        file1.open("nodes1.txt");
        file2.open("nodes2.txt");

        for (int ii = 0; ii < nodes.size(); ii++) {
            file1 << nodes[ii](0) << " " << nodes[ii](1) << " " << nodes[ii](2) << endl;
        }

        for (int ii = 0; ii < nodes2.size(); ii++) {
            file2 << nodes2[ii](0) << " " << nodes2[ii](1) << " " << nodes2[ii](2) << endl;
        }

        file1.close();
        file2.close();
    }

    for(int i = 0; i < int(nodes.size()); i++)
    {
        Vector3d coord = nodes[i];
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = coord(2);
        Points.points.push_back(pt);
        Line.points.push_back(pt);
    }
    _RRTstar_path_vis_pub.publish(Points);
    _RRTstar_path_vis_pub.publish(Line); 
}

// 显示B样条曲线的效果
void visRRTstarPathBSpline(vector<Vector3d> nodes){
    vector<Vector3d> bspline = generate_b_spline_path(nodes);
    if (bspline.size() <= 0) { 
        ROS_INFO("[node] using RRT");
        nodes = interpolation_to_RRT(nodes);
        nodes = select_waypoint(nodes);
        droneStatus->setNewRoute(nodes); 
        return; 
    } // 没有结果
    else { 
        ROS_INFO("[node] using bspline");
        Vector3d start_pt = nodes[0], end_pt = nodes[nodes.size() - 1];
        bspline = smooth_b_spline_path(bspline, start_pt, end_pt);
        bspline = select_waypoint(bspline, droneStatus->getExpectedVelocity());
        droneStatus->setNewRoute(bspline); 
    }

    if (debug_path){
        ofstream file1, file2;
        file1.open("nodes.txt");
        file2.open("bspline.txt");

        for (int ii = 0; ii < nodes.size(); ii++) {
            file1 << nodes[ii](0) << " " << nodes[ii](1) << " " << nodes[ii](2) << endl;
        }

        for (int ii = 0; ii < bspline.size(); ii++) {
            file2 << bspline[ii](0) << " " << bspline[ii](1) << " " << bspline[ii](2) << endl;
        }

        file1.close();
        file2.close();
    }
    ROS_INFO("[node] Showing bspline");

    visualization_msgs::Marker points, lines;

    points.header.frame_id = map_link.c_str();
    points.header.stamp = ros::Time::now();
    points.ns = "demo_node/BSplinePath";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.id = 0;
    points.type = visualization_msgs::Marker::POINTS;
    points.scale.x = _resolution; 
    points.scale.y = _resolution;
    points.color.g = 1.0f; // 绿色点
    points.color.a = 1.0;

    lines.header.frame_id = map_link.c_str();
    lines.header.stamp = ros::Time::now();
    lines.ns = "demo_node/BSplinePath";
    lines.action = visualization_msgs::Marker::ADD;
    lines.pose.orientation.w = 1.0;
    lines.id = 0;
    lines.type = visualization_msgs::Marker::LINE_STRIP;
    lines.scale.x = _resolution / 2; 
    lines.scale.y = _resolution / 2;
    lines.color.r = 1.0f; // 红色线
    lines.color.a = 1.0;

    geometry_msgs::Point pt;
    for (int ii = 0; ii < bspline.size(); ii++) {
        Vector3d coord = bspline[ii];
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = coord(2);
        points.points.push_back(pt);
        lines.points.push_back(pt);
    }
    _b_spline_path_vis_pub.publish(points);
    _b_spline_path_vis_pub.publish(lines);
    ROS_INFO("[node] bspline showing complete");
}

