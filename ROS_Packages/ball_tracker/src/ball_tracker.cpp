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
static const std::string HSV_WINDOW = "HSV View";

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

    // Conversion to HSV
    cv::Mat hsv_image;
    cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);

    // Displaying image
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::imshow(HSV_WINDOW, hsv_image);
    cv::waitKey(3);
}

int main(int argc, char **argv)
{
    // Node initialization
    ros::init(argc, argv, "ball_tracker");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("ardrone/front/image_raw", 1, imageCallback);

    // Creating windows
    cv::namedWindow(OPENCV_WINDOW);
    cv::namedWindow(HSV_WINDOW);

    ros::spin();
}
