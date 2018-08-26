#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <eigen3/Eigen/Dense>

ros::Publisher pub_odom;

void callback(const geometry_msgs::PoseStamped::ConstPtr& msg_pose, const geometry_msgs::TwistStamped::ConstPtr& msg_twist) {
	if( msg_pose->header.frame_id == msg_twist->header.frame_id ) {
		nav_msgs::Odometry msg_out;

		msg_out.header = msg_pose->header;

		msg_out.pose.pose = msg_pose->pose;

		//TODO: twist

		pub_odom.publish(msg_out);
	} else {
		ROS_WARN_THROTTLE( 2.0, "[%s] Frame header mismatch, dropping messages", ros::this_node::getName().c_str() );
	}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "odom_sync");

	ros::NodeHandle nh;
	ros::NodeHandle nhp( "~" );

	pub_odom = nh.advertise<nav_msgs::Odometry>("odom", 10);

	std::string param_source_ns("~");
	nhp.param("source_namespace", param_source_ns, param_source_ns );
	ros::NodeHandle nhn = ros::NodeHandle(param_source_ns);

	message_filters::Subscriber<geometry_msgs::PoseStamped> sub_pose(nhn, "pose", 1);
	message_filters::Subscriber<geometry_msgs::TwistStamped> sub_twist(nhn, "twist", 1);

	message_filters::TimeSynchronizer<geometry_msgs::PoseStamped, geometry_msgs::TwistStamped> sub_sync(sub_pose, sub_twist, 10);
	sub_sync.registerCallback( boost::bind( &callback, _1, _2 ) );

	ros::spin();

	return 0;
}
