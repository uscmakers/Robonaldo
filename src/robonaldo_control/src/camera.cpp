#include "ros/ros.h"
#include "std_msgs/String.h"


#include "opencv2/opencv.hpp"
using namespace cv;


// Reads from camera_images topic, publishes to vision_results
void userInputCallback(const std_msgs::String::ConstPtr& msg) {

}

int main(int argc, char **argv) {
	ros::init(argc, argv, "camera");
	ros::NodeHandle n;
	ros::Publisher vision_results_pub = n.advertise<std_msgs::String>("vision_results", 1000);
	ros::Subscriber sub = n.subscribe("camera_images", 1000, userInputCallback);
	ros::Rate loop_rate(10);

    VideoCapture cam("nvcamerasrc ! nvvidconv ! ximagesink");
    // open the default camera, use something different from 0 otherwise;
    // Check VideoCapture documentation.
    while (!cam.isOpened()) {
        std::cout << "Failed to make connection to cam" << std::endl;
        cam.open(0);
    }
    
	int count = 0;
	while (ros::ok()) {
		// This is a message object. stuff it with data and publish it

         Mat frame;
          cam >> frame;
          if( frame.empty() ) break; // end of video stream
          imshow("this is you, smile! :)", frame);
          if( waitKey(10) == 27 ) break; // stop capturing by pressing ESC 
  
		std_msgs::String vision_results_msg;


		ROS_INFO("Sending %f %f", 0.0, 0.0);

		vision_results_pub.publish(vision_results_msg);

		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}
	return 0;
}
