#!/bin/sh
roslaunch px4 mavros_posix_sitl.launch & sleep 5
#roslaunch vins vins_rviz.launch & sleep 3;
rosrun vins vins_node ~/catkin_ws/src/VINS-Fusion/config/realsense_d435i/realsense_stereo_imu_config.yaml & sleep 3
rosrun VtD VtD & sleep 3
#roslaunch vins rtabmap_vins.launch & sleep 3;
roslaunch rtabmap_ros rtabmap.launch & sleep 1
#roslaunch navigation_drone run_rrt.launch & sleep 3;
rosrun navigation_drone rrt_node

#rosrun tf static_transform_publisher 0 0 0 -1.57 0 -1.57 base_link camera_link 30 # ypr
rosrun mavros mavcmd long 511 105 1000 0 0 0 0 0



