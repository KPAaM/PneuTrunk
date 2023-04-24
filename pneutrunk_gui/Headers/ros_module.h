#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <pneutrunk_msgs/msg/pneutrunk_joint_state.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "cv_bridge/cv_bridge.h"
#include <opencv2/highgui.hpp>

#include <QWidget>
#include <QThread>
#include <QtCore>

using namespace std::chrono_literals;
using std::placeholders::_1;

class rosModule : public QThread
{
    Q_OBJECT
    public:
        explicit rosModule(QObject *parent = 0);
        void run();
        void JointStateCallback(const pneutrunk_msgs::msg::PneutrunkJointState &msg);
        void CameraGestureCallback(const sensor_msgs::msg::Image &msg);
        void ObjectDetectionCallback(const sensor_msgs::msg::Image &msg);

        const uint NUM_DOF = 7;
        std::vector<std::pair<double, double>> joints_actual = std::vector<std::pair<double, double>>(7);
        cv::Mat camera_gesture_matrix;
        cv::Mat camera_object_detection_matrix;
    
    private:
        rclcpp::executors::SingleThreadedExecutor _executor;
        rclcpp::Node::SharedPtr _node;
        rclcpp::Subscription<pneutrunk_msgs::msg::PneutrunkJointState>::SharedPtr _state_subscriber;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _camera_gesture_subscriber;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _camera_object_detection_subscriber;


    signals:
        void rosUpdate();
        void rosGestureUpdate();
        void rosObjectDetectionUpdate();

};