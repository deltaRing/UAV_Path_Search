cmake_minimum_required(VERSION 3.0.2)
project(navigation_drone)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  octomap_msgs
  mavros_msgs
  sensor_msgs
  nav_msgs
  std_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(ompl REQUIRED)
find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)

###################################
## catkin specific configuration ##
###################################

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS} 
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES VtD
  CATKIN_DEPENDS geometry_msgs pcl_conversions pcl_ros roscpp rospy octomap_msgs mavros_msgs sensor_msgs nav_msgs std_msgs
  DEPENDS system_lib
)

link_libraries(${OMPL_LIBRARY_DIRS})

add_executable(rrt_node 
	rrt_demo.cpp
	src/rrt.cpp
  src/pid.cpp
  src/drone_move.cpp
  src/callback_functions.cpp
  src/bspline_generate.cpp
  src/path_optimal/path_optimal.cpp
  )

## Specify libraries to link a library or executable target against
target_link_libraries(rrt_node
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${OMPL_LIBRARIES}
   ${Sophus_LIBRARIUES}
   ${CERES_LIBRARIES} 
)
