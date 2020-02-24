#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
 
// Reads from camera_images topic, publishes to vision_results

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter 
{
	ros::NodeHandle n;
	image_transport::ImageTransport it;
	image_transport::Subscriber image_sub;
	// ros::Publisher image_pub;

	public:
		ImageConverter() : it(n) {
			image_sub = it.subscribe("camera_images", 1, &ImageConverter::image_callback, this);
			// image_pub = n.advertise<std_msgs::String>("vision_results", 1);
			cv::namedWindow(OPENCV_WINDOW);
		}

		~ImageConverter() {
			cv::destroyWindow(OPENCV_WINDOW);
		}

		void image_callback(const sensor_msgs::ImageConstPtr& msg) {
			cv_bridge::CvImagePtr cv_ptr;
			std_msgs::String pub_msg;

			try {
				cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
			}
			catch (cv_bridge::Exception& e) {
				ROS_ERROR("cv_bridge exception: %s", e.what());
				return;
			}

			pub_msg.data = "success";
			// image_pub.publish(pub_msg);
		}
};



int main(int argc, char **argv) {
	ros::init(argc, argv, "vision");
	ros::Rate loop_rate(10);

	int count = 0;
	while (ros::ok()) {

		ImageConverter ic;

		ROS_INFO("Sending %f %f", 0.0, 0.0);

		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}
	return 0;
}
