#include <ros/ros.h>
#include <nodelet/loader.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <image_transport/image_transport.h>
#include <iostream>
#include <stdio.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
using namespace cv;
using namespace std;
#include <string>
#include <vector>

class ObjectPlacement {

    public:
        ObjectPlacement(ros::NodeHandle& _nh);
        ~ObjectPlacement();


        void image_callback(const sensor_msgs::ImageConstPtr& msg);

    private:
        ros::NodeHandle nh;
        image_transport::ImageTransport image_transport;
        image_transport::Subscriber image_subscriber;
        image_transport::Publisher image_test_pub;

        std::string input_image_topic;

        //blue thresholding vectors
        std::vector<int> blue_min_threshold_1;
        std::vector<int> blue_max_threshold_1;

        std::vector<int> blue_min_threshold_2;
        std::vector<int> blue_max_threshold_2;
        float min_area;


};

ObjectPlacement::ObjectPlacement(ros::NodeHandle& _nh):
        nh(_nh),
        image_transport(_nh)
    {
        //private config

        nh.getParam("object_placement/input_camera_topic", input_image_topic);
        image_subscriber = image_transport.subscribe(input_image_topic, 1,
                                               &ObjectPlacement::image_callback, this);


        image_test_pub = image_transport.advertise("/overlay_test", 1);

        //min range
        nh.getParam("/object_placement/blue_threshold/min_threshold_1", blue_min_threshold_1);
        nh.getParam("/object_placement/blue_threshold/max_threshold_1", blue_max_threshold_1);

        //higher range
        nh.getParam("/object_placement/blue_threshold/min_threshold_2", blue_min_threshold_2);
        nh.getParam("/object_placement/blue_threshold/max_threshold_2", blue_max_threshold_2);

        nh.param<float>("/object_placement/min_area", min_area);
    }

ObjectPlacement::~ObjectPlacement() {}


void ObjectPlacement::image_callback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat image = cv_ptr->image;
    cv::Mat dst = image.clone();
    cv::Mat hsv;

    cv::Mat banana_image_colour = imread("/home/talin/ar_turtlebot_racing/src/object_placement/src/banana.png");
    cv::Mat banana_image = imread("/home/talin/ar_turtlebot_racing/src/object_placement/src/banana.png",IMREAD_UNCHANGED);

    // Rect ROI (x, y, width, height);
    Rect ROI (50, 50, 200, 200);

    // Mat channel[4];
    vector<Mat> channel;

    split(banana_image, channel);

    Mat rgb[3] = { channel[0],channel[1],channel[2] };
    Mat mask_layer = channel[3]; // png's alpha channel used as mask

    merge(rgb, 3, banana_image);  // put together the RGB channels, now transp insn't transparent
    int yPos = 10;
    int xPos = 10;
    banana_image.copyTo(image.rowRange(yPos, yPos + banana_image.rows).colRange(xPos, xPos + banana_image.cols), mask_layer);

    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

    cv::Mat mask1, mask2;
    cv::inRange(hsv, cv::Scalar(blue_min_threshold_1[0], blue_min_threshold_1[1], blue_min_threshold_1[2]),
                cv::Scalar(blue_max_threshold_1[0], blue_max_threshold_1[1], blue_max_threshold_1[2]), mask1);

    cv::inRange(hsv, cv::Scalar(blue_min_threshold_2[0], blue_min_threshold_2[1], blue_min_threshold_2[2]),
                cv::Scalar(blue_max_threshold_2[0], blue_max_threshold_2[1], blue_max_threshold_2[2]), mask2);

    cv::Mat mask;
    cv::bitwise_or(mask1, mask2, mask);

    //close small holes
    cv::Mat morph_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5), cv::Point(-1,-1));
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, morph_kernel);

    //now get blobs
    cv::Mat canny_output;
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;

    //canny detect edges
    cv::Canny(mask, canny_output, 100, 150, 3);

    //find contours
    cv::findContours(canny_output, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    //now get moments
    std::vector<cv::Moments> moments(contours.size());
    std::vector<cv::Point2f> centroids(contours.size());

    std::vector<cv::Point2f> points_list;
    for (size_t i = 0; i < contours.size(); i++) {

        double area = cv::contourArea(contours[i]);

        if (area > min_area) {

            moments[i] = cv::moments(contours[i]);

            //add 1e-5 tp avoid division by zero
            centroids[i] = cv::Point2f(static_cast<float>(moments[i].m10/ (moments[i].m00 + 1e-5)),
                                       static_cast<float>(moments[i].m01/ (moments[i].m00 + 1e-5)));

            //draw circle on image
           cv::circle(dst, centroids[i], 10, cv::Scalar(0,255,0));
           points_list.push_back(centroids[i]);

        }
    }

    sensor_msgs::ImagePtr img_msg; // >> message to be sent

    img_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", mask).toImageMsg();

    img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

    image_test_pub.publish(img_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_placement");
    ros::NodeHandle n;

    ObjectPlacement object_placement(n);
    ros::spin();

}
