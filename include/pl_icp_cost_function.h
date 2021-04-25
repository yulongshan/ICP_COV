#ifndef PL_ICP_COST_FUNCTION_H
#define PL_ICP_COST_FUNCTION_H

#include <iostream>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <ceres/dynamic_autodiff_cost_function.h>
#include <Eigen/Eigen>
using namespace std;
// cost function : \sum|(Rq_i + t - p_i)^Tn_i|^2
struct PLICP
{
    PLICP(const Eigen::Vector2d q, const Eigen::Vector2d p1, const Eigen::Vector2d p2)
        :q(q),p1(p1),p2(p2){}

    template <typename T>
    bool operator()(const T* const pose, T* residuals)const
    {
        T p_q[2]; 
        T R[2][2];
        R[0][0] = cos(pose[2]);
        R[0][1] = -sin(pose[2]);
        R[1][0] = sin(pose[2]);
        R[1][1] = cos(pose[2]);
        p_q[0] = T(q(0));
        p_q[1] = T(q(1));
        T p_pro[2];
        p_pro[0] = R[0][0] * T(q(0)) + R[0][1] * T(q(1));
        p_pro[1] = R[1][0] * T(q(0)) + R[1][1] * T(q(1));

        p_pro[0] += pose[0];
        p_pro[1] += pose[1];

        T normal_vector[2];
        normal_vector[0] = T(-(p1(1) - p2(1)));
        normal_vector[1] = T(p1(0) - p2(0));
        T norm;
        norm = sqrt(normal_vector[0]*normal_vector[0] + normal_vector[1]*normal_vector[1]);
        normal_vector[0] = normal_vector[0] / norm;
        normal_vector[1] = normal_vector[1] / norm;

        T diff[2];
        diff[0] = p_pro[0] - T(p1(0));
        diff[1] = p_pro[1] - T(p1(1));
        
        residuals[0] = diff[0] * normal_vector[0] + diff[1] * normal_vector[1];
        return true;
    }
    
    static ceres::CostFunction *create(const Eigen::Vector2d q, const Eigen::Vector2d p1, const Eigen::Vector2d p2)
    {
        return(new ceres::AutoDiffCostFunction<PLICP,1,3>(new PLICP(q,p1,p2)));
    }
    Eigen::Vector2d q;
    Eigen::Vector2d p1;
    Eigen::Vector2d p2;

};

#endif
