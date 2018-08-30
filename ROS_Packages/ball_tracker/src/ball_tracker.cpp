#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    ROS_DEBUG("Image received.");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ball_tracker");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("ardrone/front/image_raw", 1, imageCallback);
    ros::spin();
}
