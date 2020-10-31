#include <ros/ros.h>
#include <nodelet/loader.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseStamped.h>
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

        //gets the postion and orientation of the turtlebot in the world frame
        geometry_msgs::PoseStamped get_turtlebot_pose(cv::Mat& src);

    private:
        ros::NodeHandle nh;
        image_transport::ImageTransport image_transport;
        image_transport::Subscriber image_subscriber;
        image_transport::Publisher image_test_pub;
        image_transport::Publisher image_mask_pub;

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


        image_test_pub = image_transport.advertise("/map_analyse/map_test", 10);
        image_mask_pub = image_transport.advertise("/map_analyse/map_mask", 10);
        // nh.param<std::string>("/realtime_vdo_slam/topic_prefix", topic_prefix, "/gmsl/");

        //min range
        nh.getParam("/map_analyse/blue_threshold/min_threshold_1", blue_min_threshold_1);
        nh.getParam("/map_analyse/blue_threshold/max_threshold_1", blue_max_threshold_1);

        //higher range
        nh.getParam("/map_analyse/blue_threshold/min_threshold_2", blue_min_threshold_2);
        nh.getParam("/map_analyse/blue_threshold/max_threshold_2", blue_max_threshold_2);
        // nh.param<std::vector<int>>("/map_analyse/blue_threshold/min_threshold", blue_min_threshold);
        // nh.param<std::vector<int>>("/map_analyse/blue_threshold/max_threshold", blue_max_threshold);

        nh.param<float>("/map_analyse/min_area", min_area, 100);



    }

MapAnalyse::~MapAnalyse() {}

geometry_msgs::PoseStamped MapAnalyse::get_turtlebot_pose(cv::Mat& src) {

    cv::Mat dst = src.clone();
    cv::Mat hsv;
    cv::Mat rgb;

    cv::cvtColor(src, hsv, cv::COLOR_BGR2HSV);

    //adapt thresh only on saturation
    std::vector<cv::Mat> channels(3);
    cv::Mat s_adapted, h_adapted;
    cv::Mat hsv_adapted(src.rows, src.cols, CV_8UC3);
    cv::split(hsv, channels);

    cv::adaptiveThreshold(channels[1],s_adapted,255,cv::ADAPTIVE_THRESH_GAUSSIAN_C,cv::THRESH_BINARY,17,0);

    std::vector<cv::Mat> channels_adapted;
    channels_adapted.push_back(channels[0]);
    channels_adapted.push_back(s_adapted);
    channels_adapted.push_back(channels[2]);

    cv::merge(channels_adapted, hsv_adapted);

    // cv::cvtColor(src, rgb, cv::COLOR_BGR2RGB);
    // ROS_INFO_STREAM("here");

    cv::Mat mask1, mask2;
    cv::inRange(hsv_adapted, cv::Scalar(blue_min_threshold_1[0], blue_min_threshold_1[1], blue_min_threshold_1[2]),
                cv::Scalar(blue_max_threshold_1[0], blue_max_threshold_1[1], blue_max_threshold_1[2]), mask1);

    cv::inRange(hsv_adapted, cv::Scalar(blue_min_threshold_2[0], blue_min_threshold_2[1], blue_min_threshold_2[2]),
                cv::Scalar(blue_max_threshold_2[0], blue_max_threshold_2[1], blue_max_threshold_2[2]), mask2);

    // ROS_INFO_STREAM("here1");
    
    cv::Mat mask;
    cv::bitwise_or(mask1, mask2, mask);

    //close small holes
    cv::Mat morph_kernel_open = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5), cv::Point(-1,-1));
    cv::Mat morph_kernel_close = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(12,12), cv::Point(-1,-1));

    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, morph_kernel_open);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, morph_kernel_close);

    // cv::Mat hit_miss_kernal = (cv::Mat_<int>(3, 3) <<
    //     0, 1, 0,
    //     1, -1, 1,
    //     0, 1, 0);

    // cv::morphologyEx(mask, mask, cv::MORPH_HITMISS, hit_miss_kernal);


// 
    // ROS_INFO_STREAM("here2");

    //now smooth using gaussian
    // cv::GaussianBlur(mask, mask, cv::Size(3,3), 0);

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

    cv::RNG rng(12345);

    std::vector<cv::Point2f> points_list;
    std::vector<cv::RotatedRect> minimum_rectangles( contours.size() );
    for (size_t i = 0; i < contours.size(); i++) {

        double area = cv::contourArea(contours[i]);

        if (area > min_area) {

            moments[i] = cv::moments(contours[i]);

            //add 1e-5 tp avoid division by zero
            centroids[i] = cv::Point2f(static_cast<float>(moments[i].m10/ (moments[i].m00 + 1e-5)),
                                       static_cast<float>(moments[i].m01/ (moments[i].m00 + 1e-5)));

            //draw civector<RotatedRect> minRect( contours.size() );rcle on image
            cv::circle(dst, centroids[i], 10, cv::Scalar(0,255,0));

            cv::Scalar color = cv::Scalar(rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
            // cv::drawContours(dst, contours, (int)i, color, 2, cv::LINE_8, hierarchy, 0 );
           points_list.push_back(centroids[i]);


            cv::RotatedRect rotated_rec = minAreaRect( contours[i]);
            minimum_rectangles.push_back(rotated_rec);


            cv::Point2f rect_points[4];
            rotated_rec.points(rect_points);
            // draw rotatedRect
            for (int j = 0; j < 3; j++) {
                cv::line( dst, rect_points[j], rect_points[(j+1)%4], color,4);
            }

        }
    }

    sensor_msgs::ImagePtr img_msg; // >> message to be sent
    // img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", hsv).toImageMsg();
    // image_test_pub.publish(img_msg);
    img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", dst).toImageMsg();
    image_test_pub.publish(img_msg);


    sensor_msgs::ImagePtr img_mask_msg  = cv_bridge::CvImage(std_msgs::Header(), "mono8", mask).toImageMsg();
    image_mask_pub.publish(img_mask_msg);

    geometry_msgs::PoseStamped pose;

    //TODO analyse pose, convert to world frame

    return pose;
}


void MapAnalyse::image_callback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat image = cv_ptr->image;

    auto pose = get_turtlebot_pose(image);
    
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