#include "pneutrunk_tools/continuum_kinematics.h"


Configuration::Configuration(/* args */)
{
}


///////////////////////////////////////////////////////////////////////
/// @brief Computes Denavit-Hartenberg matrix for the purpose of forward kinematics
/// @param a -> distance between the z_i and z_{i-1} axes
/// @param d -> signed distance between the x_i and x_{i-1} axes
/// @param alpha -> angle between the z_i and z_{i-1} axes
/// @param theta -> angle between the x_i and x_{i-1} axes (rotation about the z_i axis)
///////////////////////////////////////////////////////////////////////
Eigen::Matrix4d 
Configuration::DH_matrix(const double &a, const double &d, 
          const double &alpha, const double &theta)
{
    Eigen::MatrixXd T(4,4); 
    double sin_alpha = sin(alpha), cos_alpha = cos(alpha);
    double sin_theta = sin(theta), cos_theta = cos(theta);
    T << cos_theta,              -sin_theta,                   0,                a,
         sin_theta*cos_alpha,    cos_theta*cos_alpha,          -sin_alpha,       -sin_alpha*d,
         sin_theta*sin_alpha,    cos_theta*sin_alpha,          cos_alpha,        cos_alpha*d,
         0,                      0,                            0,                1;
    return T;
}

///////////////////////////////////////////////////////////////////////
/// @brief Computes Forward kinematics by multiplying DH matrices
/// @param q -> joint angles
///////////////////////////////////////////////////////////////////////
void
Configuration::ForwardKinematics(const Eigen::VectorXd &q)  //todo change q to [2 uhly]
{
    // a, d, alpha, theta
    // TODO: check
    // alpha, theta su nase uhly,
    // d -> vzdialenost medzi segmentmi
    // a -> vzdy 0?
    T1 = DH_matrix(0,          0.333,      0,            q(0));
    T2 = DH_matrix(0,          0,          -M_PI_2,      q(1));
    T3 = DH_matrix(0,          0.316,      M_PI_2,       q(2));
    T4 = DH_matrix(0,          0,          M_PI_2,       q(3));
    T5 = DH_matrix(0,          0.384,      -M_PI_2,      q(4));
    T6 = DH_matrix(0,          0,          M_PI_2,       q(5));
    T7 = DH_matrix(0,          0,          M_PI_2,       q(6));
    EEF = T1*T2*T3*T4*T5*T6*T7;
}    


