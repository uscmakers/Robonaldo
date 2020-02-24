#include "ros/ros.h"
#include "robonaldo/odometry.h"
#include "robonaldo/encoder_data.h"
#include <sstream>

float X = 0.0;
float Y = 0.0;
float THETA = 0.0;
float VELOCITY = 0.0;

void odomCallback(const robonaldo::encoder_data::ConstPtr& msg) {
  //ROS_INFO('ENCODER DATA: %f, %f, %f, %f', msg->left_count, msg->right_count,
          //msg->left_velocity, msg->right_velocity);
  X = msg->left_count * msg->left_velocity;
  Y = msg->right_count * msg->right_velocity;
  THETA = msg->left_count * msg->left_velocity - msg->right_count * msg->right_velocity;
  VELOCITY = (msg->left_velocity + msg->right_velocity) / 2;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "odometry");
  ros::NodeHandle n;
  ros::Subscriber encoder_sub = n.subscribe("encoder_data", 1000, odomCallback);
  ros::Publisher odom_pub = n.advertise<robonaldo::odometry>("odometry", 1000);
  ros::Rate loop_rate(10);
  int count = 0;

	while (ros::ok()) {
		// This is a message object. stuff it with data and publish it
		robonaldo::odometry msg;
		msg.x = X;
		msg.y = Y;
    msg.theta = THETA;

		ROS_INFO("ODOMETRY_PUBLISHER: Sending x=%f, y=%f, theta=%f",msg.x, msg.y, msg.theta);

		odom_pub.publish(msg);

		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}

	return 0;
}
