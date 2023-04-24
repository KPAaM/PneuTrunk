#include "pneutrunk_tools/continuum_model.h"


ContinuumModel::ContinuumModel(/* args */)
    : Node("pneutrunk_model")
{
    _timer = this->create_wall_timer(LOOP_TIME, std::bind(&ContinuumModel::Update, this));
    _cable_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("/pneutrunk/cables/state", 1);
    _state_subscriber = this->create_subscription<pneutrunk_msgs::msg::PneutrunkJointState>("/pneutrunk/joints/state", 
                        1, std::bind(&ContinuumModel::StateCallback, this, _1));


    _cable_markers_ptr = new visualization_msgs::msg::MarkerArray[1];
    this->InitCableMarkers();

    // _joint_state.segment_state.resize(_NUMBER_OF_SEGMENTS);

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
    // Prepare data
    Eigen::VectorXd q(13);
    q << _joint_state.segment_state[0], _joint_state.segment_state[1],
         _joint_state.segment_state[2], _joint_state.segment_state[3],
         _joint_state.segment_state[4], _joint_state.segment_state[5],
         _joint_state.segment_state[6], _joint_state.segment_state[7],
         _joint_state.segment_state[8], _joint_state.segment_state[9],
         _joint_state.segment_state[10], _joint_state.segment_state[11],
         _joint_state.translation;
    robot.ForwardKinematics(q);

    // Update rotation disk 1-6
    for (uint i=0; i<_NUMBER_OF_SEGMENTS; ++i)
    {
        // label link
        std::string segment_label = "segment_";
        segment_label += std::to_string(i+1);

        // fill transform msg
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = parent_frame.c_str();
        t.child_frame_id = segment_label.c_str();

        t.transform.translation.x = (robot.Seg_T[i](0,3)/1000.0);
        t.transform.translation.y = (robot.Seg_T[i](1,3)/1000.0);
        t.transform.translation.z = (robot.Seg_T[i](2,3)/1000.0);


        Eigen::Quaterniond quat(robot.Seg_T[i].topLeftCorner<3,3>());
        t.transform.rotation.x = quat.x();
        t.transform.rotation.y = quat.y();
        t.transform.rotation.z = quat.z();
        t.transform.rotation.w = quat.w();

        // Send the transformation
        _tf_broadcaster->sendTransform(t);
    }


    // Publish cables 
    // TODO: compute position
    // Eigen::Vector3d eePc;
    // const double segPhi = 0.0;
    // const double segKappa = 0.0001;
    // const double segmentLength = 1.274;
    // for (uint i=0; i<_RESOLUTION; ++i)
    // {
    //     eePc(0) = cos(segPhi)*(cos(segKappa*((i/((double)_RESOLUTION-1))*segmentLength)) - 1)/segKappa;
	// 	eePc(1) = sin(segPhi)*(cos(segKappa*((i/((double)_RESOLUTION-1))*segmentLength)) - 1)/segKappa;
	// 	eePc(2) = (sin(segKappa*((i/((double)_RESOLUTION-1))*segmentLength))/segKappa);

    //     eePc = robot.Seg_T[0].topLeftCorner<3,3>()*eePc;

    //     _cable_markers_ptr[0].markers[i].pose.position.x = robot.Seg_T[0](0,3)/1000.0 + eePc(0);
    //     _cable_markers_ptr[0].markers[i].pose.position.y = robot.Seg_T[0](1,3)/1000.0 + eePc(1);
    //     _cable_markers_ptr[0].markers[i].pose.position.z = robot.Seg_T[0](2,3)/1000.0 + eePc(2);
    //     _cable_markers_ptr[0].markers[i].pose.orientation.x = 0.0;
    //     _cable_markers_ptr[0].markers[i].pose.orientation.y = 0.0;
    //     _cable_markers_ptr[0].markers[i].pose.orientation.z = 0.0;
    //     _cable_markers_ptr[0].markers[i].pose.orientation.w = 1.0;
    // }
    // _cable_publisher->publish(*_cable_markers_ptr);
}