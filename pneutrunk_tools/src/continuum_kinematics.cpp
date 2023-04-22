#include "pneutrunk_tools/continuum_kinematics.h"


Configuration::Configuration(/* args */)
{
    T.resize(7);
    Seg_T.resize(7);
}


///////////////////////////////////////////////////////////////////////
/// @brief Computes transformation matrix for one segment
///////////////////////////////////////////////////////////////////////
Eigen::Matrix4d 
Configuration::SegmentTransform(const double &roll, const double &pitch)
{
    double roll_rad = roll*M_PI/180.0;
    double pitch_rad = pitch*M_PI/180.0;
    // Transform from prev plate to middle (potentiometer level)
    Eigen::MatrixXd T_base_to_poten(4,4);   
    T_base_to_poten << 1, 0, 0, 0,
                       0, 1, 0, 0,
                       0, 0, 1, _r,
                       0, 0, 0, 1;
    // Rotation around roll axis
    Eigen::MatrixXd T_rot_roll(4,4);  
    T_rot_roll << cos(roll_rad), 0, sin(roll_rad), 0,
                  0, 1, 0, 0,
                  -sin(roll_rad), 0, cos(roll_rad), 0,
                  0, 0, 0, 1;
    // Rotation around pitch axis
    Eigen::MatrixXd T_rot_pitch(4,4);
    T_rot_pitch << 1, 0, 0, 0,
               0, cos(pitch_rad), -sin(pitch_rad), 0,
               0, sin(pitch_rad), cos(pitch_rad), 0,
               0, 0, 0, 1;
    // Translation from middle to next plate
    Eigen::MatrixXd T_poten_to_base2(4,4);
    T_poten_to_base2 << 1, 0, 0, 0,
                        0, 1, 0, 0,
                        0, 0, 1, _r,
                        0, 0, 0, 1;
    return T_base_to_poten*T_rot_roll*T_rot_pitch*T_poten_to_base2;
}

///////////////////////////////////////////////////////////////////////
/// @brief Computes Forward kinematics 
/// @param q -> vector of joint angles
///////////////////////////////////////////////////////////////////////
void
Configuration::ForwardKinematics(const Eigen::VectorXd &q)  //todo change q to [2 uhly]
{
    T[0] = SegmentTransform(q(0),q(1));
    T[1] = SegmentTransform(q(2),q(3));
    T[2] = SegmentTransform(q(4),q(5));
    T[3] = SegmentTransform(q(6),q(7));
    T[4] = SegmentTransform(q(8),q(9));
    T[5] = SegmentTransform(q(10),q(11));
    T[6] << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 2*_r+q(12),
            0, 0, 0, 1;
    

    Seg_T[0] = T[0];
    Seg_T[1] = Seg_T[0]*T[1];
    Seg_T[2] = Seg_T[1]*T[2];
    Seg_T[3] = Seg_T[2]*T[3];
    Seg_T[4] = Seg_T[3]*T[4];
    Seg_T[5] = Seg_T[4]*T[5];
    Seg_T[6] = Seg_T[5]*T[6];
}    


