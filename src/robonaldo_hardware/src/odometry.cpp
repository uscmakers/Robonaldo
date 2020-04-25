#include "ros/ros.h"
#include "robonaldo_msgs/encoder_values.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sstream>
#include <math.h>

//Robot width: 11", Wheel radius: 4"
//convert 11 inches to encoder ticks:
//11 *()
const double ROBOT_WIDTH = 11 ; //NEED TO CHANGE 
const double inches_per_tick = 0.0006223; // 8 * 0.0254 * 3.14 / 1024 = 0.0245 (circumference over # of ticks per rotation)

double X = 0.0;
double Y = 0.0;
double TH = 0.0;
double VX = 0.0;
double VY = 0.0;
double VTH = 0.0;
double last_left_count = 0.0;
double last_right_count = 0.0;

void odomCallback(const robonaldo_msgs::encoder_values::ConstPtr& msg) {
  //convert msg data (left and right count) to inches with inches_per_tick
  double delta_left_count = (msg->left_count  - last_left_count) * inches_per_tick;
  double delta_right_count = (msg->right_count - last_right_count) * inches_per_tick;

  double delta_dist = (delta_left_count + delta_right_count) / 2.0;
  double delta_theta = (delta_right_count - delta_left_count)/ROBOT_WIDTH;

  double forward_velocity = (msg->left_velocity + msg->right_velocity)*(inches_per_tick) / 2.0;
  //CCW is positive angle
  VTH = (msg->left_velocity - msg->right_velocity)*(inches_per_tick)/ROBOT_WIDTH;

  X += ( delta_dist * cos(delta_theta) );
  Y += ( delta_dist * sin(delta_theta) );

  TH += delta_theta;

  VX = forward_velocity * cos(TH);
  VY = forward_velocity * sin(TH);

  last_left_count = msg->left_count;
  last_right_count = msg->right_count;
  
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "odometry");
  ros::NodeHandle n;
  ros::Subscriber encoder_sub = n.subscribe("encoder_values", 1000, odomCallback);
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odometry", 1000);
  ros::Rate loop_rate(10);

  tf::TransformBroadcaster odom_broadcaster;
  ros::Time current_time;

	while (ros::ok()) {

    current_time = ros::Time::now();

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(TH);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = X;
    odom_trans.transform.translation.y = Y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = X;
    odom.pose.pose.position.y = Y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = VX;
    odom.twist.twist.linear.y = VY;
    odom.twist.twist.angular.z = VTH;

		odom_pub.publish(odom);

		ros::spinOnce();

		loop_rate.sleep();
	}

	return 0;
}
