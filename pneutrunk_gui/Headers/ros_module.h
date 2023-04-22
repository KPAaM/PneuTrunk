#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <pneutrunk_msgs/msg/pneutrunk_joint_state.hpp>

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

        const uint NUM_DOF = 7;
        std::vector<std::pair<double, double>> joints_actual = std::vector<std::pair<double, double>>(7);
    private:
        rclcpp::executors::SingleThreadedExecutor _executor;
        rclcpp::Node::SharedPtr _node;
        rclcpp::Subscription<pneutrunk_msgs::msg::PneutrunkJointState>::SharedPtr _state_subscriber;
    

    signals:
        void rosUpdate();

};