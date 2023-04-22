#include "ros_module.h"

rosModule::rosModule(QObject *parent)
    : QThread(parent)
{
}

void rosModule::run()
{
    _node = std::make_shared<rclcpp::Node>("pneutrunk_gui_subscriber");
    _state_subscriber = _node->create_subscription<pneutrunk_msgs::msg::PneutrunkJointState>("/continuum_robot/state", 
                        1, std::bind(&rosModule::JointStateCallback, this, _1));
    _executor.add_node(_node);
    _executor.spin();
}

void rosModule::JointStateCallback(const pneutrunk_msgs::msg::PneutrunkJointState &msg)
{
    // Copy rotation joints
    for (uint i=0; i<(NUM_DOF-1); i++)
    {
        joints_actual[i].first = msg.segment_state[i*2];
        joints_actual[i].second = msg.segment_state[i*2+1];
    }
    // copy translation of the last joint
    joints_actual[NUM_DOF-1].first = msg.translation;

    emit rosUpdate();
}