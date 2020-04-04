#include "ros/ros.h"
#include "robonaldo_hardware/imu_values.h"
#include "robonaldo/orientation.h"
#include "MadgwickAHRS.h"

//do all the math here plz 
void userInputCallback(const robonaldo_hardware::imu_values::ConstPtr& msg){
	ROS_INFO("ax: %d ay: %d az: %d mx: %d my: %d mz: %d gx: %d gy: %d gz: %d", msg->ax, msg->ay, msg->az, msg->mx, msg->my, msg->mz, msg->gx, msg->gy, msg->gz);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "fused_orientation");
	ros::NodeHandle n;
	ros::Publisher orientation_pub = n.advertise<robonaldo::orientation>("orientation", 1000);
	ros::Subscriber sub = n.subscribe("imu_values", 1000, userInputCallback);
	ros::Rate loop_rate(10);
	
  Madgwick filter;

	const float dps_to_rad=.0174533;

	while (ros::ok()) {

		//Since I am stupid for now we will just use values
		// that I set because C++ and ROS confuse me because I am a failure
		// but also I want to convert the values
		// int axRaw = imu_values.ax, ayRaw = imu_values.ay, azRaw = imu_values.az; // 3-axis raw acceleration
  //       int gxRaw = imu_values.gx, gyRaw = imu_values.gy, gzRaw = imu_values.gz; // 3-axis raw angular velocity
  //       int mxRaw = imu_values.mx, myRaw = imu_values.my, mzRaw = imu_values.mz;

  //       //convert velocity from degrees per second to radians per second 
  //       float gxRads=gxRaw*dps_to_rad;
  //       float gyRads=gyRaw*dps_to_rad;
  //       float gzRads=gzRaw*dps_to_rad;

        float yawRad=0;
        float pitchRad=0;
        float rollRad=0;

        //update function to implement the sensor fusion algorithm 

        // filter.update(yawRad, pitchRad, rollRad, 
        //           axRaw, ayRaw, azRaw, 
        //           gxRads, gyRads, gzRads, 
        //           mxRaw, myRaw, mzRaw);

		robonaldo::orientation msg;
		msg.yaw = yawRad * 57.295779513f;
		msg.pitch = pitchRad * 57.295779513f;
		msg.roll = rollRad * 57.295779513f;

		orientation_pub.publish(msg);

		ros::spinOnce();

		loop_rate.sleep();
	}
	return 0;
}
