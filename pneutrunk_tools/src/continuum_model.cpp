#include "pneutrunk_tools/continuum_model.h"


ContinuumModel::ContinuumModel(/* args */)
    : Node("continuum_model")
{
    _timer = this->create_wall_timer(LOOP_TIME, std::bind(&ContinuumModel::Update, this));
    _cable_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("continuum_robot/cables", 1);
    _state_subscriber = this->create_subscription<pneutrunk_msgs::msg::PneutrunkJointState>("continuum_robot/state", 
                        1, std::bind(&ContinuumModel::StateCallback, this, _1));


    _cable_markers_ptr = new visualization_msgs::msg::MarkerArray[1];
    this->InitCableMarkers();

    _joint_state.segment_state.resize(_NUMBER_OF_SEGMENTS);

    // Initialize the transform broadcaster
    _tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

ContinuumModel::~ContinuumModel()
{
}


void ContinuumModel::InitCableMarkers()
{
    _cable_markers_ptr[0].markers.resize(_RESOLUTION);
    for (uint i=0; i<_RESOLUTION; ++i)
    {
        _cable_markers_ptr[0].markers[i].header.frame_id = parent_frame.c_str();
        _cable_markers_ptr[0].markers[i].header.stamp = _global_time.now();
        _cable_markers_ptr[0].markers[i].ns = "basic_shapes";
	    _cable_markers_ptr[0].markers[i].id = i;
        _cable_markers_ptr[0].markers[i].type = visualization_msgs::msg::Marker::SPHERE;
        _cable_markers_ptr[0].markers[i].action = visualization_msgs::msg::Marker::ADD;
        _cable_markers_ptr[0].markers[i].scale.x = 0.1;
        _cable_markers_ptr[0].markers[i].scale.y = 0.1;
        _cable_markers_ptr[0].markers[i].scale.z = 0.1;
        _cable_markers_ptr[0].markers[i].color.a = 1.0f;
        _cable_markers_ptr[0].markers[i].color.b = 1.0;
        // _cable_markers_ptr[0].markers[i].lifetime = 0; // TODO: nejde, najdi variantu
    }
}


void ContinuumModel::StateCallback(const pneutrunk_msgs::msg::PneutrunkJointState &msg)
{
    _joint_state = msg;
}

void ContinuumModel::Update()
{
    // Update disk
    for (uint i=0; i<_NUMBER_OF_SEGMENTS; ++i)
    {
        // label link
        std::string segment_label = "segment_";
        segment_label += std::to_string(i+1);

        //
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = parent_frame.c_str();
        t.child_frame_id = segment_label.c_str();

        t.transform.translation.x = 0.5;
        t.transform.translation.y = 0;
        t.transform.translation.z = i*0.5;

        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        // q.setRPY(_joint_state.segment_state[i].roll, _joint_state.segment_state[i].phi, 0);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        // Send the transformation
        _tf_broadcaster->sendTransform(t);
    }

    // Publish cables 
    // TODO: compute position
    for (uint i=0; i<_RESOLUTION; ++i)
    {
        _cable_markers_ptr[0].markers[i].pose.position.x = 0.0;
        _cable_markers_ptr[0].markers[i].pose.position.y = 0.0;
        _cable_markers_ptr[0].markers[i].pose.position.z = 0.5;
        _cable_markers_ptr[0].markers[i].pose.orientation.x = 0.0;
        _cable_markers_ptr[0].markers[i].pose.orientation.y = 0.0;
        _cable_markers_ptr[0].markers[i].pose.orientation.z = 0.0;
        _cable_markers_ptr[0].markers[i].pose.orientation.w = 1.0;
    }
    _cable_publisher->publish(*_cable_markers_ptr);
}