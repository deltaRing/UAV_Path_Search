#include "path_optimal/path_optimal.h"

// 计算光滑项系数
double get_cost_smooth(std::vector<Eigen::Vector3d> path) {
    int path_size = path.size(); double Js = 0.0;
     double dt = 0.3333333;

    if (path_size < 3) {

    }else{
        std::vector<Eigen::Vector3d> V, A, J;
        for (int ii = 0; ii < path_size - 1; ii++) {
            V.push_back((path[ii + 1] - path[ii]) / dt);
        }

        for (int ii = 0; ii < path_size - 2; ii++) {
            A.push_back((V[ii+1] - V[ii]) / dt);
            Js += ((V[ii+1] - V[ii]) / dt).norm();
        }

        for (int ii = 0; ii < path_size - 3; ii++) {
            J.push_back((A[ii+1] - A[ii]) / dt);
            Js += ((A[ii+1] - A[ii]) / dt).norm();
        }
    }
    return Js;
}

// 计算碰撞系数
// 输入1：路径点Q (1 x 3)
// 输入2：障碍物顶点P (N x 3)
// 输入3：障碍物原有的顶点向量V (N x 3)
// 输出1：Jc 碰撞项惩罚
double get_cost_slam(std::vector<Eigen::Vector3d> Q, std::vector<ObstacleNode> obstacles){
    double Jc = 0.0;
    double Sf = 3.0;
    for (int ii = 0; ii < Q.size(); ii++){
        ObstacleNode obs = obstacles[ii];
        for (int jj = 0; jj < obs.P.size(); jj++) {
            Eigen::Vector3d Dij_temp = -(Q[ii] - obs.P[jj]);
            double Dij =  Dij_temp(0) * obs.V[jj](0) +
                                            Dij_temp(1) * obs.V[jj](1) +
                                            Dij_temp(2) * obs.V[jj](2);
            double Cij = Sf - Dij; // 计算Cost
            double Jcij = 0.0;
            if (Cij <= 0.0){

            }else if (Cij > 0.0 && Cij <= Sf){
                Jcij = pow(Cij, 3);
            }else if (Cij > Sf) {
                Jcij = 3 * Sf * pow(Cij, 2) - 3 * pow(Sf, 2) * Cij + pow(Sf, 3);
            }
            Jc += Jcij;
        }
    }
    return Jc;
}

// 计算动力可行性
// 输入1：路径Q
// 输出1：代价值
double get_cost_dynamic(std::vector<Eigen::Vector3d> Q){
    int sSeqs = Q.size();
    double dt = 0.3333333;
    double Cost = 0, Cj = 3, Cm = 10, lambda = 1e-3;
    double a1 = 0.5, a2 = 1.0, b1 = 0.5, b2 = 1.0, c1 = 2.0, c2 = 1.0; // 多项式系数
    std::vector<Eigen::Vector3d> V, A, J;
    // 求解速度
    for (int ii = 0; ii < sSeqs - 1; ii++) {
        Eigen::Vector3d V_ = (Q[ii + 1] - Q[ii]) / dt;
        V.push_back(V_);
    }
    for (int jj = 0; jj < V.size(); jj++) {
        for (int kk = 0; kk <= 2; kk++) {
            double Cr = V[jj](kk) ;
            if (Cr < -Cj) Cost += a1 * Cr * Cr + b1 * Cr + c1;
            else if (Cr >= -Cj && Cr <= -lambda * Cm) Cost += pow (-lambda * Cm - Cr, 3);
            else if (-lambda * Cm <= Cr && Cr <= lambda * Cm) { }
            else if (lambda * Cm < Cr && Cr <= Cj) Cost += pow(Cr - lambda * Cm, 3);
            else if (Cr > Cj) Cost += a2 * Cr * Cr + b2 * Cr + c2;
        }
    } 
    // 求解加速度
    for (int ii = 0; ii < sSeqs - 2; ii++) {
        Eigen::Vector3d A_ = (V[ii + 1] - V[ii]) / dt;
        A.push_back(A_);
    }
    for (int jj = 0; jj < A.size(); jj++) {
        for (int kk = 0; kk <= 2; kk++) {
            double Cr = A[jj](kk) ;
            if (Cr < -Cj) Cost += a1 * Cr * Cr + b1 * Cr + c1;
            else if (Cr >= -Cj && Cr <= -lambda * Cm) Cost += pow (-lambda * Cm - Cr, 3);
            else if (-lambda * Cm <= Cr && Cr <= lambda * Cm) { }
            else if (lambda * Cm < Cr && Cr <= Cj) Cost += pow(Cr - lambda * Cm, 3);
            else if (Cr > Cj) Cost += a2 * Cr * Cr + b2 * Cr + c2;
        }
    } 
    // 求解加加速度
    for (int ii = 0; ii < sSeqs - 3; ii++) {
        Eigen::Vector3d J_ = (A[ii + 1] - A[ii]) / dt;
        J.push_back(J_);
    }
    for (int jj = 0; jj < J.size(); jj++) {
        for (int kk = 0; kk <= 2; kk++) {
            double Cr = J[jj](kk) ;
            if (Cr < -Cj) Cost += a1 * Cr * Cr + b1 * Cr + c1;
            else if (Cr >= -Cj && Cr <= -lambda * Cm) Cost += pow (-lambda * Cm - Cr, 3);
            else if (-lambda * Cm <= Cr && Cr <= lambda * Cm) { }
            else if (lambda * Cm < Cr && Cr <= Cj) Cost += pow(Cr - lambda * Cm, 3);
            else if (Cr > Cj) Cost += a2 * Cr * Cr + b2 * Cr + c2;
        }
    }

    return Cost;
}

// 找到周边障碍物以及障碍物向量
// 输入1：路径Q
// 输入2：地图Map
// 输入3：周边半径R
// 输出1：障碍物顶点P
// 输出2：障碍物向量V
void FindObstacleVector(std::vector<Eigen::Vector3d> path, class RRTstarPreparatory * Map, double R, std::vector<ObstacleNode> & obstacles){
    if (Map == NULL) return;
    double _resolution = Map->getResolution();
    for (int pp = 0; pp < path.size(); pp++) {
        Eigen::Vector3d _path_ = path[pp];
        ObstacleNode node;
        for (double x_range = -R; x_range < R; x_range += _resolution){
            for (double y_range = -R; y_range < R; y_range += _resolution){
                for (double z_range = -R; z_range < R; z_range += _resolution){
                    Eigen::Vector3d _pt_ = _path_ + Eigen::Vector3d(x_range, y_range, z_range);
                    if (!Map->isObsFree(_pt_(0), _pt_(1), _pt_(2), 0, 0, 0)){
                        Eigen::Vector3d Vec(x_range, y_range, z_range);
                        Vec = Vec / Vec.norm();
                        node.P.push_back(_pt_);
                        node.V.push_back(Vec);
                    }
                }
            }
        }
        obstacles.push_back(node);
    }
}

// 优化路线
void optimizePath(std::vector<Eigen::Vector3d> & path, std::vector<ObstacleNode> nodes){
    const int path_size = path.size();
    if (_MINIMAL_PTS_ > path_size) {
        ROS_ERROR("[optimal] need more points,expected %d, current: %d", _MINIMAL_PTS_, path_size);
        return;
    }
    const int compared_path = 5; // 用于计算的path
    ROS_WARN("[node] optimizing path");

    ceres::Problem problem;
    double ** path_ = new double * [path_size - compared_path + 1];
    for (int ii = 0; ii < path_size - compared_path + 1; ii++) {
        path_[ii] = new double[compared_path * 3];
        std::vector<ObstacleNode> part_nodes;
        for (int iii = 0; iii < compared_path; iii++) {
            path_[ii][iii * 3 + 0] = path[ii + iii](0);
            path_[ii][iii * 3 + 1]  = path[ii + iii](1);
            path_[ii][iii * 3 + 2]  = path[ii + iii](2);
            part_nodes.push_back(nodes[ii + iii]);
        }
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<ObsCost, 1,  compared_path * 3>(new ObsCost(compared_path, part_nodes)), NULL, path_[ii]
        );      
    }  
    // 配置求解器并求解，输出结果
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    for (int iii = 0; iii < path_size - compared_path + 1; iii++) {
        Eigen::Vector3d Q_(path_[iii][0], path_[iii][1], path_[iii][2]);
        path[iii] = Q_;
    }
    ROS_WARN("[node] optimize complete");

    for (int ii = 0; ii < path_size - compared_path + 1; ii++) {
        delete [] path_[ii];
    }
    delete [] path_;
}