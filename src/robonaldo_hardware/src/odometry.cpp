#include "ros/ros.h"
#include "robonaldo/odometry.h"
#include "robonaldo/encoder_values.h"
#include <sstream>
#include <math.h>

//Robot width: 11", Wheel radius: 4"
//convert 11 inches to encoder ticks:
//11 *()
const float ROBOT_WIDTH = 11 ; //NEED TO CHANGE 
const float inches_per_tick = 0.0245f; // 8*3.14 / 1024 = 0.0245 (circumference over # of ticks per rotation)

float X = 0.0;
float Y = 0.0;
float THETA = 0.0;
float FORWARD_VEL = 0.0;
float ANGULAR_VEL = 0.0;
float last_left_count = 0.0;
float last_right_count = 0.0;

void odomCallback(const robonaldo::encoder_values::ConstPtr& msg) {
  //convert msg data (left and right count) to inches with inches_per_tick
  float delta_left_count = (msg->left_count  - last_left_count) * inches_per_tick;
  float delta_right_count = (msg->right_count - last_right_count) * inches_per_tick;

  float delta_dist = (delta_left_count + delta_right_count) / 2.0f;
  float delta_theta = (delta_right_count - delta_left_count)/ROBOT_WIDTH;

  FORWARD_VEL = (msg->left_velocity + msg->right_velocity)*(inches_per_tick) / 2.0f;
  //CCW is positive angle
  ANGULAR_VEL = (msg->left_velocity - msg->right_velocity)*(inches_per_tick)/ROBOT_WIDTH;

  X += ( delta_dist * cos(delta_theta) );
  Y += ( delta_dist * sin(delta_theta) );

  THETA += delta_theta;
  last_left_count = msg->left_count;
  last_right_count = msg->right_count;
  
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "odometry");
  ros::NodeHandle n;
  ros::Subscriber encoder_sub = n.subscribe("encoder_values", 1000, odomCallback);
  ros::Publisher odom_pub = n.advertise<robonaldo::odometry>("odometry", 1000);
  ros::Rate loop_rate(10);

	while (ros::ok()) {
		// This is a message object. stuff it with data and publish it
		robonaldo::odometry msg;
		msg.x = X;
		msg.y = Y;
    msg.theta = THETA;
    msg.forward_velocity = FORWARD_VEL;
    msg.angular_velocity = ANGULAR_VEL;

		ROS_INFO("ODOMETRY_PUBLISHER: Sending x=%f, y=%f, theta=%f",msg.x, msg.y, msg.theta);

		odom_pub.publish(msg);

		ros::spinOnce();

		loop_rate.sleep();
	}

	return 0;
}
