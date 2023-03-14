#pragma once
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <functional>
#include <memory>

#include <iostream>
#include <string> 
#include "visualization_msgs/msg/marker_array.hpp"
#include <eigen3/Eigen/Dense>
#include "pneutrunk_tools/continuum_kinematics.h"
#include <pneutrunk_msgs/msg/pneutrunk_joint_state.hpp>
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

#define LOOP_TIME 100ms

class ContinuumModel : public rclcpp::Node
{
    public:
        ContinuumModel(/* args */);
        ~ContinuumModel();
    private:
        void Update();
        void InitCableMarkers();
        void StateCallback(const pneutrunk_msgs::msg::PneutrunkJointState &msg);

        rclcpp::TimerBase::SharedPtr _timer;    // timer for generating update interrupt
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _cable_publisher;
        rclcpp::Subscription<pneutrunk_msgs::msg::PneutrunkJointState>::SharedPtr _state_subscriber;
        std::unique_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;

        const uint _NUMBER_OF_SEGMENTS = 3;
        const uint _RESOLUTION = 200; // number of markers between segments
        visualization_msgs::msg::MarkerArray* _cable_markers_ptr;
        rclcpp::Clock _global_time;    
        pneutrunk_msgs::msg::PneutrunkJointState _joint_state;
};
