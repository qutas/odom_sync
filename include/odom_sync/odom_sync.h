#pragma once

#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <eigen3/Eigen/Dense>

#include <string>

class OdomSync {
	private:
		ros::NodeHandle nh_;
		ros::NodeHandle nhp_;

		message_filters::Subscriber<geometry_msgs::PoseStamped> sub_pose_;
		message_filters::Subscriber<geometry_msgs::TwistStamped> sub_twist_;
		message_filters::TimeSynchronizer<geometry_msgs::PoseStamped, geometry_msgs::TwistStamped> sub_sync_;

		std::string param_source_ns_;

	public:
		OdomSync( void );

		~OdomSync( void );

	private:
	void callback(const geometry_msgs::PoseStamped::ConstPtr& msg_pose, const geometry_msgs::TwistStamped::ConstPtr& msg_twist);
};
