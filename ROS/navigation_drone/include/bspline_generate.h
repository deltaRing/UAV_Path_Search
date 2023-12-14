#ifndef _BSPLINE_GENERATE_H_
#define _BSPLINE_GENERATE_H_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include <iostream>

using namespace std;

// 计算基函数
// 输入1：当前t
// 输入2：NodeVector 
// 输入3：k 点数
// 输入4：m 阶数
// 输出1：当前的权重
double get_weight_N(double t, Eigen::ArrayXd NodeVector, int k, int m);

// 根据OMPL所生成的路线，生成B样条曲线
// 输入1：根据OMPL所生成的路线
// 输入2：dt
// 输入3：order 几阶的B样条曲线
//输出：b样条曲线
std::vector<Eigen::Vector3d> generate_b_spline_path(std::vector<Eigen::Vector3d> Path_point, double dt = 0.005, unsigned int order = 3);

// 平滑B样条曲线
// 输入1：bspline B样条曲线
// 输入2：_start_pt_ 开始点
// 输入3：_end_pt_ 结束点
// 输入4：_init_ratio_  开始点保留阈值
// 输入5：_end_ratio_ 结束点保留阈值
// 输入6：dt 线段步进
// 输出1：theCurl 平滑后的曲线
std::vector<Eigen::Vector3d> smooth_b_spline_path(std::vector<Eigen::Vector3d> bspline, Eigen::Vector3d _start_pt_, Eigen::Vector3d _end_pt_, double _init_ratio_ = 0.05, double _end_ratio_ = 0.125);

// 给RRT路线插值
std::vector<Eigen::Vector3d> interpolation_to_RRT(std::vector <Eigen::Vector3d> RRT_node, double dx = 0.05);

#endif