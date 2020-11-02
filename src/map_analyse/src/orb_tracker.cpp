#include <ros/ros.h>
#include <nodelet/loader.h>

#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <stdio.h>


#include <string>


#include "orb_tracker.hpp"

OrbTracker::OrbTracker(const std::string& _reference_image_path) :
        reference_image_path(_reference_image_path)
    {
        orb_detector = cv::ORB::create(MAX_FEATURES);
        matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
        set_reference_image();
    }

void OrbTracker::set_reference_image() {
    reference_image = cv::imread(reference_image_path, CV_LOAD_IMAGE_GRAYSCALE);
    cv::resize(reference_image, reference_image, cv::Size(640, 480), 0, 0, CV_INTER_LINEAR);

    if (reference_image.rows == 0 || reference_image.cols == 0) {
        ROS_WARN_STREAM("Reference image " << reference_image_path << " could not be loaded");
        return;
    }
    // cv::Mat reference_img_grey;
    // cv::cvtColor(reference_image,reference_img_grey,cv::COLOR_BGR2GRAY);

    orb_detector->detectAndCompute(reference_image, cv::Mat(), reference_keypoints, reference_desriptors);
    ROS_INFO_STREAM("Loaded reference image " << reference_image_path << " and computed features");

}

cv::Mat& OrbTracker::calculate_image_alignment(cv::Mat& input) {
    cv::Mat input_img_grey;
    // cv::cvtColor(input,input_img_grey,cv::COLOR_BGR2GRAY);
    orb_detector->detectAndCompute(input, cv::Mat(), input_keypoints, input_descriptors);

    //match features
    std::vector<cv::DMatch> matches;
    matcher->match(reference_desriptors, input_descriptors, matches, cv::Mat());

    if (matches.size() > 1) {


        std::sort(matches.begin(), matches.end());

        // Remove not so good matches
        const int numGoodMatches = matches.size() * 0.15f;
        matches.erase(matches.begin()+numGoodMatches, matches.end());
        ROS_INFO_STREAM("Found " << matches.size() << " feature matches");
        
        
        // Extract location of good matches
        std::vector<cv::Point2f> points1, points2;
        ROS_INFO_STREAM("input size" << input.size());
        ROS_INFO_STREAM("reference size" << reference_image.size());

        for( size_t i = 0; i < matches.size(); i++ ) {
            points1.push_back( reference_keypoints[ matches[i].queryIdx ].pt );
            points2.push_back( input_keypoints[ matches[i].trainIdx ].pt );
        }
        
        // Find homography
        homography = cv::findHomography( points1, points2, cv::RANSAC );
        
        // Use homography to warp image
        cv::warpPerspective(input, warped_image, homography, input.size());

    }

    return warped_image;
    

}