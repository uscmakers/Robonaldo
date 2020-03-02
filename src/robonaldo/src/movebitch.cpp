#include "ros/ros.h"
#include <geometry_msgs/Twist.h>

int main(int argc, char **argv) {
	ros::init(argc, argv, "move_bitch");
	ros::NodeHandle n;
	ros::Publisher move_bitch_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
	ros::Rate loop_rate(10);

	while(ros::ok()){
		geometry_msgs::Twist msg;
		msg.linear.x = 1.0;
		move_bitch_pub.publish(msg);
		//break;
	}

	return 0;
}
