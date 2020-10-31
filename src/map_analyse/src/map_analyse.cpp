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


#include <string>

// handler.param<std::string>("/realtime_vdo_slam/topic_prefix", topic_prefix, "/gmsl/");

class MapAnalyse {

    public:
        MapAnalyse(ros::NodeHandle& _nh);
        ~MapAnalyse();


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

MapAnalyse::MapAnalyse(ros::NodeHandle& _nh):
        nh(_nh),
        image_transport(_nh) 
    {   
        //private config

        nh.getParam("map_analyse/input_camera_topic", input_image_topic);
        ROS_INFO_STREAM(input_image_topic);
        image_subscriber = image_transport.subscribe(input_image_topic, 1,
                                               &MapAnalyse::image_callback, this);


        image_test_pub = image_transport.advertise("/map_test", 1);
        // nh.param<std::string>("/realtime_vdo_slam/topic_prefix", topic_prefix, "/gmsl/");

        //min range
        nh.getParam("/map_analyse/blue_threshold/min_threshold_1", blue_min_threshold_1);
        nh.getParam("/map_analyse/blue_threshold/max_threshold_1", blue_max_threshold_1);

        //higher range
        nh.getParam("/map_analyse/blue_threshold/min_threshold_2", blue_min_threshold_2);
        nh.getParam("/map_analyse/blue_threshold/max_threshold_2", blue_max_threshold_2);
        // nh.param<std::vector<int>>("/map_analyse/blue_threshold/min_threshold", blue_min_threshold);
        // nh.param<std::vector<int>>("/map_analyse/blue_threshold/max_threshold", blue_max_threshold);

        nh.param<float>("/map_analyse/min_area", min_area);



    }

MapAnalyse::~MapAnalyse() {}


void MapAnalyse::image_callback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat image = cv_ptr->image;
    cv::Mat dst = image.clone();
    cv::Mat hsv;

    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
    // ROS_INFO_STREAM("here");

    cv::Mat mask1, mask2;
    cv::inRange(hsv, cv::Scalar(blue_min_threshold_1[0], blue_min_threshold_1[1], blue_min_threshold_1[2]),
                cv::Scalar(blue_max_threshold_1[0], blue_max_threshold_1[1], blue_max_threshold_1[2]), mask1);

    cv::inRange(hsv, cv::Scalar(blue_min_threshold_2[0], blue_min_threshold_2[1], blue_min_threshold_2[2]),
                cv::Scalar(blue_max_threshold_2[0], blue_max_threshold_2[1], blue_max_threshold_2[2]), mask2);

    // ROS_INFO_STREAM("here1");
    
    cv::Mat mask;
    cv::bitwise_or(mask1, mask2, mask);

    //close small holes
    cv::Mat morph_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5), cv::Point(-1,-1));
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, morph_kernel);
// 
    // ROS_INFO_STREAM("here2");

    //now smooth using gaussian
    // cv::GaussianBlur(mask, mask, cv::Size(5,5), 0);

    //now get blobs
    cv::Mat canny_output;
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;

    //canny detect edges
    cv::Canny(mask, canny_output, 100, 150, 3);

    //find contours
    cv::findContours(canny_output, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    //do closing to remove small holes
    //cv::morphologyEx(canny_output, canny_output, cv::MORPH_CLOSE, morph_kernel);

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
    // img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", dst).toImageMsg();
    // image_test_pub.publish(img_msg);
    img_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", mask).toImageMsg();
    image_test_pub.publish(img_msg);
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_analyse");
    ros::NodeHandle n;

    MapAnalyse map_analyse(n);
    ros::spin();

    

    // image_transport::ImageTransport it(n);
    // image_transport::Publisher pub_img = it.advertise(output_video_topic, 1);

    // cv::VideoCapture cap;

    // if (!cap.open(device_id)) {
    //     ROS_ERROR("Camera failed to open");
    //     return -1;
    // }

    // cv::Mat image;
    // sensor_msgs::ImagePtr img_msg; // >> message to be sent

    // while (ros::ok()) {
    //     cap.read(image);

    //     if (image.empty()) {
    //         ROS_WARN("Frame was empty");
    //     }
    //     else {
    //         img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    //         pub_img.publish(img_msg);
    //     }
    //     ros::spinOnce();

    // }
    // return 0;
}