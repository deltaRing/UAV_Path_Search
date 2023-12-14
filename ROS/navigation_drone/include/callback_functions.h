#ifndef _CALLBACK_FUNCTIONS_H_
#define _CALLBACK_FUNCTIONS_H_

#include <iostream>
#include <fstream>
#include <math.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <ompl/config.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/Path.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/SimpleSetup.h>

#include <geometry_msgs/PoseStamped.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/ColorOcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#define SOPHUS_USE_BASIC_LOGGING
#include <sophus/so3.hpp>
#include <sophus/se3.hpp>

// 订阅从地图转移到无人机的点云变化
#include <tf2_msgs/TFMessage.h>
// 坐标系转换
#include <pcl_ros/transforms.h>
// 用多线程来处理OMPL问题
#include <thread>

using namespace std;
using namespace Eigen;

namespace ob = ompl::base;
namespace og = ompl::geometric;

void rcvCurrentPoseCallback(const geometry_msgs::PoseStamped & pos);
void rcvWaypointsCallback(const geometry_msgs::PoseStamped & wp);
// PointCloud2 -> pcl point clouds
void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 & pointcloud_map);
ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si);
ob::OptimizationObjectivePtr getThresholdPathLengthObj(const ob::SpaceInformationPtr& si);
void pathFinding(const Vector3d start_pt, const Vector3d target_pt);
void visRRTstarPath(vector<Vector3d> nodes );
void visRRTstarPathBSpline(vector<Vector3d> nodes); // 显示B样条曲线的效果

#endif