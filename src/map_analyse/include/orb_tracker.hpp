#ifndef AR_TURTLEBOT_ORB_TRACKER
#define AR_TURTLEBOT_ORB_TRACKER



#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <opencv2/features2d.hpp>
#include <sensor_msgs/Image.h>
#include <opencv2/core.hpp>
#include "opencv2/xfeatures2d.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/core/cvstd.hpp>
#include <image_transport/image_transport.h>
#include <iostream>
#include <stdio.h>

#include <vector>
#include <string>
#include <memory>

constexpr int MAX_FEATURES = 500;

class OrbTracker {

    public:
        OrbTracker(const std::string& _reference_image_path);

        /**
         * Calculated the homography matrix between the input image and the reference image
         */
        cv::Mat& calculate_image_alignment(cv::Mat& input);



    private:

        cv::Ptr<cv::ORB> orb_detector;
        cv::Ptr<cv::DescriptorMatcher> matcher;
        cv::Mat reference_desriptors, input_descriptors;
        cv::Mat reference_image, input_image;
        std::vector<cv::KeyPoint> reference_keypoints, input_keypoints;
        // std::vector<cv::Point2f> object_bb;

        cv::Mat homography;
        cv::Mat warped_image;

        const std::string reference_image_path;
        
        /**
         * Loads the reference image from the provided path and calculated keypooints and descriptors in advanced
         */
        void set_reference_image();


};





#endif
