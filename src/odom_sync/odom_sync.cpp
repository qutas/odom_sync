#include <odom_sync/odom_sync.h>

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <eigen3/Eigen/Dense>

OdomSync::OdomSync() :
	nh_(),
	nhp_( "~" ),
	param_source_ns_("~"),
	sub_pose_(nh_, "pose", 1),
	sub_twist_(nh_, "twist", 1),
	sub_sync_(sub_pose_, sub_twist_, 10) {

	nhp_.param("source_namespace", param_source_ns_, param_source_ns_ );

	nh_ = ros::NodeHandle(param_source_ns_);

	//sub_pose_ = message_filters::Subscriber<geometry_msgs::PoseStamped>(nh_, "pose", 1);
	//sub_twist_ = message_filters::Subscriber<geometry_msgs::TwistStamped>(nh_, "twist", 1);

	sub_sync_.registerCallback( boost::bind( &OdomSync::callback, this, _1, _2 ) );
}

OdomSync::~OdomSync() {
}

void OdomSync::callback(const geometry_msgs::PoseStamped::ConstPtr& msg_pose, const geometry_msgs::TwistStamped::ConstPtr& msg_twist) {
	if( msg_pose->header.frame_id == msg_twist->header.frame_id ) {

	} else {
		ROS_WARN_THROTTLE( 2.0, "[%s] Frame header mismatch, dropping messages", ros::this_node::getName().c_str() );
	}
}
