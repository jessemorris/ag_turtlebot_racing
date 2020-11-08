#ifndef AR_TURTLEBOT_MAP_ANALYSE
#define AR_TURTLEBOT_MAP_ANALYSE

#include <ros/ros.h>
#include <nodelet/loader.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseStamped.h>
#include <opencv2/features2d.hpp>
#include <sensor_msgs/Image.h>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/cvstd.hpp>
#include <image_transport/image_transport.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <iostream>
#include <stdio.h>

#include <vector>
#include <string>
#include <memory>


#include "turtlebot.hpp"
// #include "orb_tracker.hpp"



class MapAnalyse {

    public:
        MapAnalyse(ros::NodeHandle& _nh);
        ~MapAnalyse();


        void image_callback(const sensor_msgs::ImageConstPtr& msg);

        //gets the postion and orientation of the turtlebot in the world frame
        bool get_turtlebot_pose(cv::Mat& src,  geometry_msgs::PoseStamped& pose_stamped);


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

        std::vector<int> red_min_threshold_1;
        std::vector<int> red_max_threshold_1;
        float min_area;


        std::unique_ptr<Turtlebot> turtlebot;
        // std::unique_ptr<OrbTracker> orb_tracker;

        tf2_ros::StaticTransformBroadcaster static_broadcster;



};




#endif
