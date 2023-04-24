#include "ros_module.h"

rosModule::rosModule(QObject *parent)
    : QThread(parent)
{
}

void rosModule::run()
{
    _node = std::make_shared<rclcpp::Node>("pneutrunk_gui_subscriber");
    _state_subscriber = _node->create_subscription<pneutrunk_msgs::msg::PneutrunkJointState>("/pneutrunk/joints/state", 
                        1, std::bind(&rosModule::JointStateCallback, this, _1));

    _camera_gesture_subscriber = _node->create_subscription<sensor_msgs::msg::Image>("/pneutrunk/gesture/camera", 
                        1, std::bind(&rosModule::CameraGestureCallback, this, _1));
    _camera_object_detection_subscriber = _node->create_subscription<sensor_msgs::msg::Image>("/pneutrunk/object_detection/camera", 
                        1, std::bind(&rosModule::ObjectDetectionCallback, this, _1));

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

void rosModule::CameraGestureCallback(const sensor_msgs::msg::Image &msg)
{
    camera_gesture_matrix = cv_bridge::toCvCopy(msg, "bgr8")->image;
    emit rosGestureUpdate();
}

void rosModule::ObjectDetectionCallback(const sensor_msgs::msg::Image &msg)
{
    camera_object_detection_matrix = cv_bridge::toCvCopy(msg, "bgr8")->image;
    emit rosObjectDetectionUpdate();
}