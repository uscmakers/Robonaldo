#include "ros/ros.h"
#include "robonaldo_hardware/imu_values.h"
#include "robonaldo/orientation.h"

void userInputCallback(const robonaldo_hardware::imu_values::ConstPtr& msg){
	ROS_INFO("ax: %d mx: %d gx: %d", msg->ax, msg->mx, msg->gx);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "fused_orientation");
	ros::NodeHandle n;
	ros::Publisher orientation_pub = n.advertise<robonaldo::orientation>("orientation", 1000);
	ros::Subscriber sub = n.subscribe("imu_values", 1000, userInputCallback);
	ros::Rate loop_rate(10);
	
	while (ros::ok()) {

		robonaldo::orientation msg;
		msg.yaw = 0.5;
		msg.pitch = 0.1;
		msg.roll = 10;

		orientation_pub.publish(msg);

		ros::spinOnce();

		loop_rate.sleep();
	}
	return 0;
}