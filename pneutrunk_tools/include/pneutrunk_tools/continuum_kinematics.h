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
        Eigen::Matrix4d SegmentTransform(const double &roll, const double &pitch);

        void ForwardKinematics(const Eigen::VectorXd &q);

        std::vector<Eigen::Matrix4d> T;
        std::vector<Eigen::Matrix4d> Seg_T;
    private:
        double _r = -62.0;

};


