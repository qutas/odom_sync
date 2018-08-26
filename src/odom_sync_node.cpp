#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

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

		//Build a world-body rotation matrix
		Eigen::Matrix3d rot = Eigen::Quaterniond(msg_pose->pose.orientation.w,
												 msg_pose->pose.orientation.x,
												 msg_pose->pose.orientation.y,
												 msg_pose->pose.orientation.z).normalized().toRotationMatrix().transpose();

		Eigen::Vector3d vl = rot*Eigen::Vector3d(msg_twist->twist.linear.x,
												 msg_twist->twist.linear.y,
												 msg_twist->twist.linear.z);
		Eigen::Vector3d va = rot*Eigen::Vector3d(msg_twist->twist.angular.x,
												 msg_twist->twist.angular.y,
												 msg_twist->twist.angular.z);

		msg_out.twist.twist.linear.x = vl.x();
		msg_out.twist.twist.linear.y = vl.y();
		msg_out.twist.twist.linear.z = vl.z();
		msg_out.twist.twist.angular.x = va.x();
		msg_out.twist.twist.angular.y = va.y();
		msg_out.twist.twist.angular.z = va.z();

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

	typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, geometry_msgs::TwistStamped> ApproxPolicy;
	message_filters::Synchronizer<ApproxPolicy> sub_sync(ApproxPolicy(10), sub_pose, sub_twist);
	sub_sync.registerCallback( boost::bind( &callback, _1, _2 ) );

	ros::spin();

	return 0;
}
