#include "ros/ros.h"
#include "robonaldo_msgs/imu_values.h"
#include "robonaldo_msgs/orientation.h"
#include "MadgwickAHRS.h"

//am I allowed to make global variables? we will see 
float yawRad=0;
float pitchRad=0;
float rollRad=0;

const float dps_to_rad=.0174533;

Madgwick filter;

void userInputCallback(const robonaldo_msgs::imu_values::ConstPtr& msg){
	ROS_INFO("ax: %d ay: %d az: %d mx: %d my: %d mz: %d gx: %d gy: %d gz: %d", msg->ax, msg->ay, msg->az, msg->mx, msg->my, msg->mz, msg->gx, msg->gy, msg->gz);
	float axRaw = msg->ax, ayRaw = msg->ay, azRaw = msg->az; // 3-axis raw acceleration
	float gxRaw = msg->gx, gyRaw = msg->gy, gzRaw = msg->gz; // 3-axis raw angular velocity
	float mxRaw = msg->mx, myRaw = msg->my, mzRaw = msg->mz;
	
//convert velocity from degrees per second to radians per second 
	float gxRads=gxRaw*dps_to_rad;
	float gyRads=gyRaw*dps_to_rad;
	float gzRads=gzRaw*dps_to_rad;

        //update function to implement the sensor fusion algorithm 

        filter.update(
                   axRaw, ayRaw, azRaw, 
                   gxRads, gyRads, gzRads, 
                   mxRaw, myRaw, mzRaw);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "fused_orientation");
	ros::NodeHandle n;
	ros::Publisher orientation_pub = n.advertise<robonaldo_msgs::orientation>("orientation", 1000);
	ros::Subscriber sub = n.subscribe("imu_values", 1000, userInputCallback);
	ros::Rate loop_rate(10);
	
	while (ros::ok()) {

//the 57 number is the number to convert radians to degrees i think 
		robonaldo_msgs::orientation msg;
		msg.yaw = filter.getYaw();
		msg.pitch = filter.getPitch();
		msg.roll = filter.getRoll();

		orientation_pub.publish(msg);

		ros::spinOnce();

		loop_rate.sleep();
	}
	return 0;
}
