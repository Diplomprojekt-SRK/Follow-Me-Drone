// ROS includes
#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
// OpenCV includes
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Ball Tracker";

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    // Image conversion
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    // Displaying image
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);
}

int main(int argc, char **argv)
{
    cv::namedWindow(OPENCV_WINDOW);
    ros::init(argc, argv, "ball_tracker");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("ardrone/front/image_raw", 1, imageCallback);
    ros::spin();
}
