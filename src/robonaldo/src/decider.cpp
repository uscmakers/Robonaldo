#include "ros/ros.h"
#include "robonaldo_msgs/keyboard_input.h"
#include "robonaldo_msgs/motor_speeds.h"

const float MAX_RIGHT_SPEED = 1.0f;
const float MIN_RIGHT_SPEED = 1.0f;
const float MAX_LEFT_SPEED = 1.0f;
const float MIN_LEFT_SPEED = 1.0f;

const float KEYBOARD_SPEED = 0.25f;

robonaldo_msgs::motor_speeds motor_speeds_msg;

void userInputCallback(const robonaldo_msgs::keyboard_input::ConstPtr& msg){

	//ROS_INFO("Recieved %u %u %u %u", msg->up, msg->down, msg->left, msg->right);

	motor_speeds_msg.left_speed = 0.0f;
	motor_speeds_msg.right_speed = 0.0f;

	if(msg->up == true) {
		motor_speeds_msg.left_speed += KEYBOARD_SPEED;
		motor_speeds_msg.right_speed += KEYBOARD_SPEED;
	}

	if(msg->down == true) {
		motor_speeds_msg.left_speed += -KEYBOARD_SPEED;
		motor_speeds_msg.right_speed += -KEYBOARD_SPEED;
	}

	if(msg->left == true) {
		motor_speeds_msg.left_speed += -KEYBOARD_SPEED;
		motor_speeds_msg.right_speed += KEYBOARD_SPEED;
	}

	if(msg->right == true) {
		motor_speeds_msg.left_speed += KEYBOARD_SPEED;
		motor_speeds_msg.right_speed += -KEYBOARD_SPEED;
	}

}

int main(int argc, char **argv) {
	ros::init(argc, argv, "decider");
	ros::NodeHandle n;
	ros::Publisher motor_control_pub = n.advertise<robonaldo_msgs::motor_speeds>("motor_control", 1000);
	ros::Subscriber sub = n.subscribe("user_input", 1000, userInputCallback);
	ros::Rate loop_rate(60);

	motor_speeds_msg.left_speed = 0.0;
	motor_speeds_msg.right_speed = 0.0;

	while (ros::ok()) {
		// This is a message object. stuff it with data and publish it

		//ROS_INFO("Sending %f %f", motor_speeds_msg.left_speed, motor_speeds_msg.right_speed);

		motor_control_pub.publish(motor_speeds_msg);

		ros::spinOnce();

		loop_rate.sleep();
	}
	return 0;
}
