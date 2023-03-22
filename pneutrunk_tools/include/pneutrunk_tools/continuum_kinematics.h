#pragma once
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <functional>
#include <memory>

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <vector>

class Configuration
{
    public:
        Configuration(/* args */);
        Eigen::Matrix4d DH_matrix(const double &a, const double &d, 
                                const double &alpha, const double &theta);

        void ForwardKinematics(const Eigen::VectorXd &q);    

        Eigen::Matrix4d T1, T2, T3, T4, T5, T6, T7, EEF;
    private:

};


