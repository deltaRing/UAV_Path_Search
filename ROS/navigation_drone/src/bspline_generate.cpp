#include "bspline_generate.h"

// 计算基函数
// 输入1：当前t
// 输入2：NodeVector 
// 输入3：k 点数
// 输入4：m 阶数
// 输出1：当前的权重
double get_weight_N(double t, Eigen::ArrayXd NodeVector, int k, int m) {
    double weight = 0.0;
    if (m == 0) {
            if (t >= NodeVector(k) && t <= NodeVector(k + 1)) {
                weight = 1.0;
            }else {
                weight = 0.0;
            }
    }else {
        double L1 = (NodeVector(k + m + 0) - NodeVector(k + 0));
        double L2 = (NodeVector(k + m + 1) - NodeVector(k + 1));
        if (abs(L1) < 1e-6) L1 = 1; // 规定了 0 / 0 = 0
        if (abs(L2) < 1e-6) L2 = 1; 
        double Bk_M_1   = get_weight_N(t, NodeVector, k, m - 1);
        double Bk_1_M_1 = get_weight_N(t, NodeVector, k + 1, m - 1);
        weight = (t - NodeVector(k + 0)) / L1 * Bk_M_1 +
            (NodeVector(m + k + 1) - t) / L2 * Bk_1_M_1;
    }
    return weight;
}

// 根据OMPL所生成的路线，生成B样条曲线
// 输入1：根据OMPL所生成的路线
// 输入2：dt
// 输入3：order 几阶的B样条曲线
// 输出1：b样条曲线
std::vector<Eigen::Vector3d> generate_b_spline_path(std::vector<Eigen::Vector3d> Path_point, double dt, unsigned int order){
    std::vector<Eigen::Vector3d>  new_path;

    int n = Path_point.size() - 1; if (n < order) { return new_path; }
    Eigen::ArrayXd NodeVector(n + order + 1); // 节点矢量
    NodeVector(0) = 0.0; double dii = 1.0 / (n + order);
    for (int ii = 1; ii <= n + order; ii++) {
        NodeVector(ii) = dii * ii;
    }

    new_path.push_back(Path_point[0]);
    for (double t = double(order - 1) / (n + order + 1); t  < double(n+2) / (n + order + 1); t+=dt ){
        // 预先定义数据
        Eigen::MatrixXd pts(n + 1, 3);
        Eigen::MatrixXd Bs(1, n + 1);
        for (int tt = 0; tt <= n; tt++) {
            Eigen::Vector3d pt = Path_point[tt];
            double B = get_weight_N(t, NodeVector, tt, order - 1);
            pts.row(tt) = pt;
            Bs(0, tt) = B;
        }
        double xxx = (Bs * pts.col(0))(0 ,0),
                        yyy = (Bs * pts.col(1))(0 ,0),
                        zzz = (Bs * pts.col(2))(0 ,0);
        new_path.push_back(Eigen::Vector3d(xxx, yyy, zzz));
    }
    new_path.push_back(Path_point[Path_point.size() - 1]);

    return new_path;
}

// 平滑B样条曲线
// 输入1：bspline B样条曲线
// 输入2：_start_pt_ 开始点
// 输入3：_end_pt_ 结束点
// 输入4：_init_ratio_  开始点保留阈值
// 输入5：_end_ratio_ 结束点保留阈值
// 输入6：dt 线段步进
// 输出1：theCurl 平滑后的曲线
std::vector<Eigen::Vector3d> smooth_b_spline_path(std::vector<Eigen::Vector3d> bspline, Eigen::Vector3d _start_pt_, Eigen::Vector3d _end_pt_, double _init_ratio_, double _end_ratio_){
    std::vector<Eigen::Vector3d> curl;

    int size_bspline = bspline.size();
    int start_size = size_bspline * _init_ratio_ + 1, end_size = (1 - _end_ratio_) * size_bspline + 1;
    // 初始化 开始的点以及结束的点
    Eigen::Vector3d designated_start_pt = bspline[start_size], designated_end_pt = bspline[end_size];

    Eigen::Vector3d dstart_pt = (designated_start_pt - _start_pt_) / start_size,
        dend_pt = (_end_pt_ - designated_end_pt) / (size_bspline - end_size);

    for (int ii = 0; ii <= start_size; ii++) {
        Eigen::Vector3d pt = _start_pt_ + dstart_pt * ii;
        curl.push_back(pt);
    }

    for (int ii = start_size; ii < end_size; ii++) {
        curl.push_back(bspline[ii]);
    }

    for (int ii = 0; ii <= size_bspline - end_size; ii++) {
        Eigen::Vector3d pt = designated_end_pt + dend_pt * ii;
        curl.push_back(pt);
    }

    return curl;
}

std::vector<Eigen::Vector3d> interpolation_to_RRT(std::vector <Eigen::Vector3d> RRT_node, double dx){
    std::vector<Eigen::Vector3d> nodes;

    for (int ii = 0; ii < RRT_node.size() - 1; ii++) {
        Eigen::Vector3d start_pt = RRT_node[ii], expected_pt = RRT_node[ii + 1];
        int steps = (expected_pt - start_pt).norm() / dx + 1;
        Eigen::Vector3d step_pt = (expected_pt - start_pt) / steps;
        for (int jj = 0; jj < steps; jj++) {
            nodes.push_back(start_pt + jj * step_pt);
        }
    }

    return nodes;
}

std::vector<Eigen::Vector3d> select_waypoint(std::vector<Eigen::Vector3d> waypoints, double distance) {
    std::vector<Eigen::Vector3d> nodes;
    Eigen::Vector3d node_start = waypoints[0];
    //nodes.push_back(node_start);
    for (int ii = 1; ii < waypoints.size(); ii++) {
        if ((node_start - waypoints[ii]).norm() > distance){
            nodes.push_back(waypoints[ii]);
            node_start = waypoints[ii];
        }
    }
    return nodes;
}