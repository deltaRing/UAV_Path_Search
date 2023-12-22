#ifndef _PATH_OPTIMAL_H_
#define _PATH_OPTIMAL_H_

// 机器人操作系统
#include <ros/ros.h>
// 优化库
#include <ceres/ceres.h>
// 矩阵运算库
#include <Eigen/Dense>
#include <Eigen/Core>
// vector
#include <vector>
// 地图定义
#include "rrt.h"
// 最小需要点数
#define _MINIMAL_PTS_ 15

// 每个路径点所对应的障碍向量
struct ObstacleNode{
    std::vector<Eigen::Vector3d> P;
    std::vector<Eigen::Vector3d> V;
};

// 计算光滑项系数 
double get_cost_smooth(std::vector<Eigen::Vector3d> path);
// 计算碰撞系数
// 输入1：路径点Q (1 x 3)
// 输入2：障碍物顶点P (N x 3)
// 输入3：障碍物原有的顶点向量V (N x 3)
// 输出1：Jc 碰撞项惩罚
double get_cost_slam(std::vector<Eigen::Vector3d> Q,  std::vector<ObstacleNode> obstacles);
// 计算动力可行性
// 输入1：路径Q
// 输出1：代价值
double get_cost_dynamic(std::vector<Eigen::Vector3d> Q);
// 找到周边障碍物以及障碍物向量
// 输入1：路径Q
// 输入2：地图Map
// 输入3：周边半径R
// 输出1：障碍物顶点P
// 输出2：障碍物向量V
void FindObstacleVector(std::vector<Eigen::Vector3d> path, class RRTstarPreparatory * Map, double R, std::vector<ObstacleNode> & obstacles);

// 障碍物代价函数
struct ObsCost{
    template <typename T>
    bool operator() (const T * const path, T * residual) const {
        residual[0] = 
        weight_smooth * get_cost_smooth_(path) + 
        weight_dynamic * get_cost_dynamic_(path) +
        weight_slam * get_cost_slam_(path);
        return true;
    }

    double weight_smooth = 1.0, weight_slam = 1.0, weight_dynamic = 1.0, dt = 0.333333333;
    double a1 = 0.5, a2 = 1.0, b1 = 0.5, b2 = 1.0, c1 = 2.0, c2 = 1.0; // 多项式系数
    double  Cj = 3, Cm = 10, lambda = 1e-1;
    double Sf = 1.0; // 安全距离
    const int path_length;
    std::vector<ObstacleNode> obstacle;

    void setA1(double a1_) { a1 = a1_; }
    void setA2(double a2_) { a2 = a2_; }
    void setB1(double b1_) { b1 = b1_; }
    void setB2(double b2_) { b2 = b2_; }
    void setC1(double c1_) { c1 = c1_; }
    void setC2(double c2_) { c2 = c2_; }
    void setCj(double cj_) { Cj = cj_; }
    void setCm(double cm_) { Cm = cm_; }
    void setlambda(double lambda_) { lambda = lambda_; } // lambda * Cm 需要小于 Cr
    void setSf(double sf_) { Sf = sf_; }
    void setWeightSmooth(double wm) { weight_smooth = wm; }
    void setWeightSlam(double ws) { weight_slam = ws; }
    void setWeightDynamic(double wd) { weight_dynamic = wd; }

    template <typename T>
    T get_cost_dynamic_(const T * const Q) const {
        T Cost = T(0.0);
        T * V = new T[path_length * 3 - 3],
        *A = new T[path_length * 3 - 6],
        *J = new T[path_length * 3 - 9];
        for (int ii = 0; ii < path_length - 1; ii++) {
            V[ii * 3] = (Q[(ii + 1) * 3] - Q[ii * 3]) / dt;
            V[ii * 3 + 1] = (Q[(ii + 1) * 3 + 1] - Q[ii * 3 + 1]) / dt;
            V[ii * 3 + 2] = (Q[(ii + 1) * 3 + 2] - Q[ii * 3 + 2]) / dt;
        }

        for (int jj = 0; jj < path_length - 1; jj++) {
            for (int kk = 0; kk <= 2; kk++) {
                T Cr = V[3 * jj + kk] ;
                if (Cr < -Cj) Cost += abs(a1 * Cr * Cr + b1 * Cr + c1);
                else if (Cr >= -Cj && Cr <= -lambda * Cm) Cost += pow (-lambda * Cm - Cr, 3);
                else if (-lambda * Cm <= Cr && Cr <= lambda * Cm) { }
                else if (lambda * Cm < Cr && Cr <= Cj) Cost += pow(Cr - lambda * Cm, 3);
                else if (Cr > Cj) Cost += a2 * Cr * Cr + b2 * Cr + c2;
            }
        } 

        for (int ii = 0; ii < path_length - 2; ii++) {
            A[ii * 3] = (V[(ii + 1) * 3] - V[ii * 3]) / dt;
            A[ii * 3 + 1] = (V[(ii + 1) * 3 + 1] - V[ii * 3 + 1]) / dt;
            A[ii * 3 + 2] = (V[(ii + 1) * 3 + 2] - V[ii * 3 + 2]) / dt;
        }

         for (int jj = 0; jj < path_length - 2; jj++) {
            for (int kk = 0; kk <= 2; kk++) {
                T Cr = A[3 * jj + kk] ;
                if (Cr < -Cj) Cost += abs(a1 * Cr * Cr + b1 * Cr + c1);
                else if (Cr >= -Cj && Cr <= -lambda * Cm) Cost += pow (-lambda * Cm - Cr, 3);
                else if (-lambda * Cm <= Cr && Cr <= lambda * Cm) { }
                else if (lambda * Cm < Cr && Cr <= Cj) Cost += pow(Cr - lambda * Cm, 3);
                else if (Cr > Cj) Cost += a2 * Cr * Cr + b2 * Cr + c2;
            }
        }

        for (int ii = 0; ii < path_length - 3; ii++) {
            J[ii * 3] = (A[(ii + 1) * 3] - A[ii * 3]) / dt;
            J[ii * 3 + 1] = (A[(ii + 1) * 3 + 1] - A[ii * 3 + 1]) / dt;
            J[ii * 3 + 2] = (A[(ii + 1) * 3 + 2] - A[ii * 3 + 2]) / dt;
        }

         for (int jj = 0; jj < path_length - 3; jj++) {
            for (int kk = 0; kk <= 2; kk++) {
                T Cr = J[3 * jj + kk] ;
                if (Cr < -Cj) Cost += abs(a1 * Cr * Cr + b1 * Cr + c1);
                else if (Cr >= -Cj && Cr <= -lambda * Cm) Cost += pow (-lambda * Cm - Cr, 3);
                else if (-lambda * Cm <= Cr && Cr <= lambda * Cm) { }
                else if (lambda * Cm < Cr && Cr <= Cj) Cost += pow(Cr - lambda * Cm, 3);
                else if (Cr > Cj) Cost += a2 * Cr * Cr + b2 * Cr + c2;
            }
        }

        std::cout << "[optimal] dynamic error: "<< Cost << std::endl;
        delete [] V, A, J;
        return Cost;
    }
    
    template <typename T>
    T get_cost_smooth_(const T * const Q) const {
        T Jd = T(0.0);
        T * V = new T[path_length * 3 - 3],
        *A = new T[path_length * 3 - 6],
        *J = new T[path_length * 3 - 9];
        for (int ii = 0; ii < path_length - 1; ii++) {
            V[ii * 3] = (Q[(ii + 1) * 3] - Q[ii * 3]) / dt;
            V[ii * 3 + 1] = (Q[(ii + 1) * 3 + 1] - Q[ii * 3 + 1]) / dt;
            V[ii * 3 + 2] = (Q[(ii + 1) * 3 + 2] - Q[ii * 3 + 2]) / dt;
            // Jd += abs(V[ii * 3] * V[ii * 3] +  V[ii * 3 + 1] * V[ii * 3 + 1] +  V[ii * 3 + 2] * V[ii * 3 + 2]);
        }

        for (int ii = 0; ii < path_length - 2; ii++) {
            A[ii * 3] = (V[(ii + 1) * 3] - V[ii * 3]) / dt;
            A[ii * 3 + 1] = (V[(ii + 1) * 3 + 1] - V[ii * 3 + 1]) / dt;
            A[ii * 3 + 2] = (V[(ii + 1) * 3 + 2] - V[ii * 3 + 2]) / dt;
            Jd += abs(A[ii * 3] * A[ii * 3] +  A[ii * 3 + 1] * A[ii * 3 + 1] +  A[ii * 3 + 2] * A[ii * 3 + 2]);
        }

        for (int ii = 0; ii < path_length - 3; ii++) {
            J[ii * 3] = (A[(ii + 1) * 3] - A[ii * 3]) / dt;
            J[ii * 3 + 1] = (A[(ii + 1) * 3 + 1] - A[ii * 3 + 1]) / dt;
            J[ii * 3 + 2] = (A[(ii + 1) * 3 + 2] - A[ii * 3 + 2]) / dt;
            Jd += abs(J[ii * 3] * J[ii * 3] +  J[ii * 3 + 1] * J[ii * 3 + 1] +  J[ii * 3 + 2] * J[ii * 3 + 2]);
        }

        std::cout << "[optimal] smooth error: "<< Jd << std::endl;
        delete [] V, A, J;
        return Jd;
    }

    template <typename T>
    T get_cost_slam_(const T * const Q) const {
        T Jc = T(0.0);
        for (int ii = 0; ii < path_length; ii++){
            ObstacleNode obs = obstacle[ii];
            for (int jj = 0; jj < obs.P.size(); jj++) {
                T Dij = -(Q[3 * ii] - T(obs.P[jj](0))) * T(obs.V[jj](0)) +
                        -(Q[3 * ii + 1] - T(obs.P[jj](1))) * T(obs.V[jj](1)) +
                        -(Q[3 * ii + 2] - T(obs.P[jj](2))) * T(obs.V[jj](2));
                T Cij = Sf - Dij; // 计算Cost
                T Jcij = T(0.0);
                if (Cij <= 0.0){

                }else if (Cij > 0.0 && Cij <= Sf){
                    Jcij = pow(Cij, 3);
               }else if (Cij > Sf) {
                    Jcij = 3 * Sf * pow(Cij, 2) - 3 * pow(Sf, 2) * Cij + pow(Sf, 3);
                }
                Jc += Jcij;
            }
        }

        std::cout << "[optimal] slam error: "<< Jc << std::endl;
        return Jc;
    }

    ObsCost(const int path_, std::vector<ObstacleNode> nodes) : path_length(path_), obstacle(nodes) {}
};

// 优化路线
void optimizePath(std::vector<Eigen::Vector3d> & path, std::vector<ObstacleNode> nodes);

#endif