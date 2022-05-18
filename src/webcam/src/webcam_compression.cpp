// Use the image_transport classes instead.
#include <ros/ros.h>
#include <image_transport/image_transport.h>

image_transport::Publisher pub = it.advertise("/webcam/compressed", 1);


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  pub.publish(msg);
}


int main(int argc, char *argv[]);
	ros::init(argc, argv, "webcam_compression");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe("/webcam/image_raw", 1, imageCallback);
	ros::spin();
}
