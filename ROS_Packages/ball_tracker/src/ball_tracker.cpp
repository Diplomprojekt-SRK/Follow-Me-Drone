// ROS includes
#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

// OpenCV includes
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Window names
static const std::string OPENCV_WINDOW = "Ball Tracker";
static const std::string BLURRED_WINDOW = "Blurred";
static const std::string HSV_WINDOW = "HSV View";
static const std::string H_WINDOW = "H Component";
static const std::string S_WINDOW = "S Component";
static const std::string V_WINDOW = "V Component";
static const std::string HT_WINDOW = "Hue Threshold";
static const std::string ST_WINDOW = "Saturation Threshold";
static const std::string VT_WINDOW = "Value Threshold";
static const std::string CT_WINDOW = "Combined Threshold";
static const std::string MORPHED_WINDOW = "Eroded and Dilated";

// Variables for threshold calibration
static const int HUE_MAX = 180;
static const int SATURATION_MAX = 255;
static const int VALUE_MAX = 255;
int hue_slider_min = 0;
int hue_slider_max = HUE_MAX;
int saturation_slider_min = 0;
int saturation_slider_max = SATURATION_MAX;
int value_slider_min = 0;
int value_slider_max = VALUE_MAX;

/*
 * Called when image is received
 */
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

    // Gaussian blur
    cv::Mat blurred;
    cv::GaussianBlur(cv_ptr->image, blurred, cv::Size(11, 11), 0);

    // Conversion to HSV
    cv::Mat hsv_image;
    cv::cvtColor(blurred, hsv_image, cv::COLOR_BGR2HSV);

    // Seperation into 3 components
    cv::Mat components[3];
    cv::split(hsv_image, components);

    // Thresholding
    cv::Mat hue_treshold;
    cv::Mat saturation_treshold;
    cv::Mat value_treshold;
    cv::Mat combined_treshold;
    cv::inRange(components[0], hue_slider_min, hue_slider_max, hue_treshold);
    cv::inRange(components[1], saturation_slider_min, saturation_slider_max, saturation_treshold);
    cv::inRange(components[2], value_slider_min, value_slider_max, value_treshold);
    cv::inRange(hsv_image, cv::Scalar(hue_slider_min, saturation_slider_min, value_slider_min), cv::Scalar(hue_slider_max, saturation_slider_max, value_slider_max), combined_treshold);

    // Erosion
    cv::Mat eroded;
    cv::erode(combined_treshold, eroded, cv::Mat(), cv::Point(-1, -1), 2, cv::BORDER_CONSTANT, cv::morphologyDefaultBorderValue());

    // Dilation
    cv::Mat dilated;
    cv::dilate(eroded, dilated, cv::Mat(), cv::Point(-1, -1), 2, cv::BORDER_CONSTANT, cv::morphologyDefaultBorderValue());

    // Circle detection
    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(dilated, circles, cv::HOUGH_GRADIENT, 2,
                 dilated.rows/16, 100, 50, 0, 0
    );

    // Drawing detected circles to screen
    for (std::size_t i = 0; i < circles.size(); ++i)
    {
            cv::Vec3i c = circles[i];
            cv::Point center = cv::Point(c[0], c[1]);
            int radius = c[2];
            // Draw circle center
            cv::circle(cv_ptr->image, center, 1, cv::Scalar(0, 255, 0), 3, cv::LINE_AA);
            // Draw circle outline
            cv::circle(cv_ptr->image, center, radius, cv::Scalar(0, 255, 0), 3, cv::LINE_AA);
    }

    // Displaying image
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::imshow(BLURRED_WINDOW, blurred);
    cv::imshow(HSV_WINDOW, hsv_image);
    cv::imshow(H_WINDOW, components[0]);
    cv::imshow(S_WINDOW, components[1]);
    cv::imshow(V_WINDOW, components[2]);
    cv::imshow(HT_WINDOW, hue_treshold);
    cv::imshow(ST_WINDOW, saturation_treshold);
    cv::imshow(VT_WINDOW, value_treshold);
    cv::imshow(CT_WINDOW, combined_treshold);
    cv::imshow(MORPHED_WINDOW, dilated);
    cv::waitKey(3);
}

/*
 * Called when user moves trackbar.
 * Nothing happens here.
 */
void trackbarCallback(int, void*)
{
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
    cv::namedWindow(BLURRED_WINDOW);
    cv::namedWindow(HSV_WINDOW);
    cv::namedWindow(H_WINDOW);
    cv::namedWindow(S_WINDOW);
    cv::namedWindow(V_WINDOW);
    cv::namedWindow(HT_WINDOW);
    cv::namedWindow(ST_WINDOW);
    cv::namedWindow(VT_WINDOW);
    cv::namedWindow(CT_WINDOW);
    cv::namedWindow(MORPHED_WINDOW);

    // Creating trackbars for threshold calibration
    cv::createTrackbar("Hue Minimum", H_WINDOW, &hue_slider_min, HUE_MAX, trackbarCallback);
    cv::createTrackbar("Hue Maximum", H_WINDOW, &hue_slider_max, HUE_MAX, trackbarCallback);
    cv::createTrackbar("Saturation Minimum", S_WINDOW, &saturation_slider_min, SATURATION_MAX, trackbarCallback);
    cv::createTrackbar("Saturation Maximum", S_WINDOW, &saturation_slider_max, SATURATION_MAX, trackbarCallback);
    cv::createTrackbar("Value Minimum", V_WINDOW, &value_slider_min, VALUE_MAX, trackbarCallback);
    cv::createTrackbar("Value Maximum", V_WINDOW, &value_slider_max, VALUE_MAX, trackbarCallback);

    ros::spin();
}
