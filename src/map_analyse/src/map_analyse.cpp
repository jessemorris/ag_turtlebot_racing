#include <ros/ros.h>
#include <nodelet/loader.h>

#include <cv_bridge/cv_bridge.h>
#include <tf2/LinearMath/Quaternion.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseStamped.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <sensor_msgs/Image.h>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <image_transport/image_transport.h>
#include <iostream>
#include <stdio.h>
#include <algorithm>


#include <string>
#include <memory>


#include "map_analyse.hpp"

#define CAMERA_WIDTH_NORM 6.4
#define CAMERA_HEIGHT_NORM 4.8


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
        nh.getParam("/map_analyse/red_threshold/min_threshold_1", red_min_threshold_1);
        nh.getParam("/map_analyse/red_threshold/max_threshold_1", red_max_threshold_1);
        // nh.param<std::vector<int>>("/map_analyse/blue_threshold/min_threshold", blue_min_threshold);
        // nh.param<std::vector<int>>("/map_analyse/blue_threshold/max_threshold", blue_max_threshold);

        nh.param<float>("/map_analyse/min_area", min_area, 100);

        // orb_tracker = std::make_unique<OrbTracker>("/home/jesse//Code/src/ar_turtlebot_racing/src/map_analyse/config/turtlebot_pose.png");

        // int rot_matrix[3][3] = [1 0 0 ; 0 -1 0 ; 0 0 -1];

        //roll around y axis to match camera frame coordinates in the top left
        tf2::Quaternion q;
        q.setRPY(0, -M_PI, 0);


        geometry_msgs::TransformStamped transform;
        transform.header.stamp = ros::Time::now();
        transform.header.frame_id = "map";
        transform.child_frame_id = "turtlebot_image_frame";

        ROS_INFO_STREAM("new thing here\n");

        transform.transform.translation.x = CAMERA_WIDTH_NORM/2.0;
        transform.transform.translation.y = -CAMERA_HEIGHT_NORM/2.0;
        transform.transform.translation.z = 0;

        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();

        static_broadcster.sendTransform(transform);

        //transform to camera coorindate math
        geometry_msgs::TransformStamped transform_1;
        transform_1.header.stamp = ros::Time::now();
        transform_1.header.frame_id = "map";
        transform_1.child_frame_id = "rotated_map";

        ROS_INFO_STREAM("new thing here\n");

        transform_1.transform.translation.x = 0;
        transform_1.transform.translation.y = 0;
        transform_1.transform.translation.z = 0;

        tf2::Quaternion q1;
        q1.setRPY(-M_PI/2.0, 0, -M_PI/2.0);

        transform_1.transform.rotation.x = q1.x();
        transform_1.transform.rotation.y = q1.y();
        transform_1.transform.rotation.z = q1.z();
        transform_1.transform.rotation.w = q1.w();

        static_broadcster.sendTransform(transform_1);



    }

MapAnalyse::~MapAnalyse() {}

bool MapAnalyse::get_turtlebot_pose(cv::Mat& src, geometry_msgs::PoseStamped& pose_stamped) {

    cv::Mat dst = src.clone();
    cv::Mat hsv;
    cv::Mat rgb;

    cv::cvtColor(src, hsv, cv::COLOR_BGR2HSV);

    cv::Mat mask1, mask2, mask;

    cv::inRange(hsv, cv::Scalar(red_min_threshold_1[0], red_min_threshold_1[1], red_min_threshold_1[2]),
                cv::Scalar(red_max_threshold_1[0], red_max_threshold_1[1], red_max_threshold_1[2]), mask);

    //close small holes
    cv::Mat morph_kernel_open = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7,7), cv::Point(-1,-1));
    cv::Mat morph_kernel_close = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(15, 15), cv::Point(-1,-1));


    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, morph_kernel_open);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, morph_kernel_close);

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
    std::vector<cv::RotatedRect> minimum_rectangles;

    cv::RNG rng(12345);
    cv::Scalar color = cv::Scalar(rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );


    std::vector<cv::Point2f> points_list;
    std::vector<cv::Vec4i> probalistic_lines;
    for (size_t i = 0; i < contours.size(); i++) {

        double area = cv::contourArea(contours[i]);

            if (area > min_area) {

            moments[i] = cv::moments(contours[i]);

            //add 1e-5 tp avoid division by zero
            centroids[i] = cv::Point2f(static_cast<float>(moments[i].m10/ (moments[i].m00 + 1e-5)),
                                       static_cast<float>(moments[i].m01/ (moments[i].m00 + 1e-5)));

            //draw civector<RotatedRect> minRect( contours.size() circle on image
            cv::circle(dst, centroids[i], 15, cv::Scalar(0,255,0));

            points_list.push_back(centroids[i]);

            cv::RotatedRect rotated_rec = cv::minAreaRect( contours[0]);
            cv::Rect original_rect = rotated_rec.boundingRect();
            float original_angle = rotated_rec.angle;
            cv::Point2f original_center = rotated_rec.center;
            cv::Size original_size(rotated_rec.size);

            cv::Size scaled_size(0.75*original_size.width,0.75*original_size.height);
            cv::RotatedRect scaled_rotated_rect(original_center,scaled_size,original_angle);

            minimum_rectangles.push_back(scaled_rotated_rect);

            cv::Point2f rect_points[4];
            scaled_rotated_rect.points(rect_points);

            //list of pairs identifying the points that make up the U on the turtlebot
            std::vector<std::pair<cv::Point2f, cv::Point2f>> u_pairs;
            std::pair<cv::Point2f, cv::Point2f> back_line;

            //draw rotatedRect
            for (int j = 0; j < 4; j++) {
                cv::Point2f point1 = rect_points[j];
                cv::Point2f point2 = rect_points[(j+1)%4];
                cv::LineIterator it(src, point1, point2, 8);
                // std::vector<cv::Vec3b> buf(it.count);
                int avg_line_value = 0;

                //iterate through each pixel in the line and find the average. If white -> add to u_pairs as part
                //of U shape on turtlebot
                for(int i = 0; i < it.count; i++, ++it) {
                    cv::Point pixel_location=  it.pos();
                    //now use pixel location to get value in mask
                    int value = mask.at<uchar>(pixel_location);
                    avg_line_value += value;

                }

                avg_line_value/= it.count;

                //for now just take if averages are over 150 pixels but this is pretty hacky
                //as this does not guarantee 3 - will need to improve search aglrithm later
                if (avg_line_value > 150) {
                    std::pair<cv::Point2f, cv::Point2f> u_line(point1, point2);
                    u_pairs.push_back(u_line);
                }
                else {
                  std::pair<cv::Point2f, cv::Point2f> u_line(point1, point2);
                  back_line = u_line;
                }
            }

            if (u_pairs.size() != 3) {
                ROS_WARN_STREAM("3 lines not found from HSV images. Rejecting this Pose.");
            }
            else {
                //two of the values should be the same so we can track if this is the one we've seen
                std::vector<int> u_angles;

                std::pair<cv::Point2f, cv::Point2f> front_line;
                for (std::pair<cv::Point2f, cv::Point2f>& line_points:u_pairs) {

                    //put everything in first quadrant
                    int angle_line = std::atan((line_points.second.y - line_points.first.y)/(line_points.second.x - line_points.first.x))*180.0/M_PI;

                    //make into ints for comparison
                    u_angles.push_back(angle_line);
                }


                if(std::abs(u_angles[0]) == 45) {
                    return false;
                }

                //lazy check
                if (u_angles[0] == u_angles[2]) {
                    //u_angles[1] is the unique one
                    front_line = u_pairs[1];
                }
                else if (u_angles[1] == u_angles[2]){
                    //u_angles[0] is the unique one
                    front_line = u_pairs[0];
                }
                else {
                    //u_angles[2] is the unique one
                    front_line = u_pairs[2];
                }

                cv::line( dst, front_line.first, front_line.second, color,4);

                //we actually just need the front line and the centoid and draw a line between the centroid and the middle of the midpoint
                float center_x_front = std::abs(back_line.first.x + back_line.second.x)/2.0;
                float center_y_front = std::abs(back_line.first.y + back_line.second.y)/2.0;
                cv::Point diff = front_line.first - front_line.second;
                int euclid_front =  cv::sqrt(diff.x*diff.x + diff.y*diff.y);

                float center_x_back = std::abs(front_line.first.x + front_line.second.x)/2.0;
                float center_y_back = std::abs(front_line.first.y + front_line.second.y)/2.0;
                diff = back_line.first - back_line.second;
                int euclid_back =  cv::sqrt(diff.x*diff.x + diff.y*diff.y);

                if(euclid_front == 0 || euclid_back == 0) {
                    ROS_WARN_STREAM("Euclid distances were 0!");
                }
                else {

                    cv::Point2f start_of_arrow(center_x_back, center_y_back);

                    cv::Point2f end_of_arrow(center_x_front, center_y_front);

                    cv::arrowedLine(dst, start_of_arrow, end_of_arrow, color, 20);

                    float y_val = (src.rows-center_y_front) - (src.rows-center_y_back);
                    float x_val = center_x_front - center_x_back;

                    float theta = 0;

                    if (x_val == 0 && y_val < 0) {
                        theta = M_PI/4;
                    }


                    // bottom left quadrant
                    else if (x_val < 0 && y_val < 0) {
                        theta = M_PI-std::atan(abs(y_val/x_val));
                    }

                    // bottom right quadrant
                    else if (x_val > 0 && y_val < 0.1) {
                        theta = std::atan(abs(y_val/x_val));
                    }

                    // top left quadrant
                    else if (x_val < 0 && y_val > 0.1) {
                        theta = std::atan(abs(y_val/x_val)) + M_PI;
                    }

                    // top right quadrant
                    else if (x_val > 0 && y_val > 0.1) {
                        theta = 2*M_PI - std::atan(abs(y_val/x_val));
                    }

                    float centre_x = (center_x_front + center_x_back)/2.0;
                    float centre_y = (center_y_front + center_y_back)/2.0;

                    tf2::Quaternion quat;
                    quat.setRPY(0, 0, theta);

                    tf2::convert(quat, pose_stamped.pose.orientation);
                    pose_stamped.pose.position.x = centre_x/100;
                    pose_stamped.pose.position.y = centre_y/100;
                    pose_stamped.pose.position.z = 0;

                    std_msgs::Header header;
                    header.frame_id = "/map";
                    header.stamp = ros::Time::now();

                    pose_stamped.header = header;
                }
            }
        }
    }


    sensor_msgs::ImagePtr img_msg; // >> message to be sent

    // publishes are for testing purposes to visualise overlays
    img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", dst).toImageMsg();
    image_test_pub.publish(img_msg);

    sensor_msgs::ImagePtr img_mask_msg  = cv_bridge::CvImage(std_msgs::Header(), "mono8", mask).toImageMsg();
    image_mask_pub.publish(img_mask_msg);

    return true;
}


void MapAnalyse::image_callback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat image = cv_ptr->image;
    geometry_msgs::PoseStamped pose;

    auto result = get_turtlebot_pose(image, pose);

    if (!result) {
        return;
    }

    if (!turtlebot) {
        turtlebot = std::make_unique<Turtlebot>(nh, pose);
    }

    else {
        turtlebot->update_pose_camera(pose);
    }
}
