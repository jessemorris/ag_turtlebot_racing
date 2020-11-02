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
#include <algorithm>


#include <string>
#include <memory>


#include "map_analyse.hpp"


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

        // ROS_INFO_STREAM("package path " << ros::package::getPath("map_analyse"));
        // orb_tracker = std::make_unique<OrbTracker>("/home/jesse//Code/src/ar_turtlebot_racing/src/map_analyse/config/turtlebot_pose.png");






    }

MapAnalyse::~MapAnalyse() {}

geometry_msgs::PoseStamped MapAnalyse::get_turtlebot_pose(cv::Mat& src) {

    cv::Mat dst = src.clone();
    cv::Mat hsv;
    cv::Mat rgb;

    cv::cvtColor(src, hsv, cv::COLOR_BGR2HSV);

    //TRIED THIS - DOES NOT WORK
    // //adapt thresh only on saturation
    // std::vector<cv::Mat> channels(3);
    // cv::Mat s_adapted, h_adapted;
    // cv::Mat hsv_adapted(src.rows, src.cols, CV_8UC3);
    // cv::split(hsv, channels);

    // cv::adaptiveThreshold(channels[1],s_adapted,255,cv::ADAPTIVE_THRESH_GAUSSIAN_C,cv::THRESH_BINARY,27,0);

    // std::vector<cv::Mat> channels_adapted;
    // channels_adapted.push_back(channels[0]);
    // channels_adapted.push_back(s_adapted);
    // channels_adapted.push_back(channels[2]);

    // cv::merge(channels_adapted, hsv_adapted);

    // cv::cvtColor(src, rgb, cv::COLOR_BGR2RGB);
    // ROS_INFO_STREAM("here");

    cv::Mat mask1, mask2, mask;
    // cv::inRange(hsv, cv::Scalar(blue_min_threshold_1[0], blue_min_threshold_1[1], blue_min_threshold_1[2]),
    //             cv::Scalar(blue_max_threshold_1[0], blue_max_threshold_1[1], blue_max_threshold_1[2]), mask);


    cv::inRange(hsv, cv::Scalar(red_min_threshold_1[0], red_min_threshold_1[1], red_min_threshold_1[2]),
                cv::Scalar(red_max_threshold_1[0], red_max_threshold_1[1], red_max_threshold_1[2]), mask);



    // cv::inRange(hsv_adapted, cv::Scalar(blue_min_threshold_2[0], blue_min_threshold_2[1], blue_min_threshold_2[2]),
    //             cv::Scalar(blue_max_threshold_2[0], blue_max_threshold_2[1], blue_max_threshold_2[2]), mask2);

    // // ROS_INFO_STREAM("here1");

    // cv::bitwise_or(mask1, mask2, mask);

    //close small holes
    cv::Mat morph_kernel_open = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3), cv::Point(-1,-1));
    cv::Mat morph_kernel_close = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(11,11), cv::Point(-1,-1));

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

            //draw civector<RotatedRect> minRect( contours.size() );rcle on image
            cv::circle(dst, centroids[i], 15, cv::Scalar(0,255,0));
            // cv::drawContours(dst, contours, (int)i, color, 2, cv::LINE_8, hierarchy, 0 );
            points_list.push_back(centroids[i]);

            // cv::HoughLinesP(mask, probalistic_lines, 1, CV_PI/180, 50, 50, 10 ); // runs the actual detection

            //must be greater than k used
            // if (probalistic_lines.size() > 2) {
            //     std::vector<float> point_angles;
            //     // Draw the lines

            //     ROS_INFO_STREAM("hough lines: " << probalistic_lines.size());
            //     for( size_t i = 0; i < probalistic_lines.size(); i++ )
            //     {
            //         cv::Vec4i l = probalistic_lines[i];
            //         cv::line(dst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), color, 3, cv::LINE_AA);
            //         //calculate line angle
            //         // float angle = std::atan2((l[1] - l[3]), (l[0] - l[2]));
            //         // point_angles.push_back(angle);


            //     }
            //     // cv::Mat best_labels, centers;
            //     // cv::kmeans(point_angles, 2, best_labels,
            //     //         cv::TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 10, 1.0),
            //     //         3, cv::KMEANS_PP_CENTERS, centers);

            //     // ROS_INFO_STREAM("size of centers " << centers.size());
            //     // //this should always be 2
            //     // for (int center = 0; center < centers.rows; center++) {
            //     //     float found_center = centers.at<float>(0, center);

            //     //     //use centroid as center for now
            //     //     float x2 = centroids[i].x + 100 * std::cos(found_center);
            //     //     float y2 = centroids[i].y + 100 * std::sin(found_center);
            //     //     cv::line(dst, cv::Point(centroids[i].x, centroids[i].y), cv::Point(x2, y2), color, 3, cv::LINE_AA);
            //     //     ROS_INFO_STREAM("results " << found_center);
            //     // }



            // }

             // std::sort(points_list.begin(), points_list.end());
            //should sort contours here

            cv::RotatedRect rotated_rec = cv::minAreaRect( contours[0]);
            cv::Rect original_rect = rotated_rec.boundingRect();
            float original_angle = rotated_rec.angle;
            cv::Point2f original_center = rotated_rec.center;
            cv::Size original_size(rotated_rec.size);


            cv::Size scaled_size(0.75*original_size.width,0.75*original_size.height);
            cv::RotatedRect scaled_rotated_rect(original_center,scaled_size,original_angle);
            // cv::Rect original_rect = rotated_rec.boundingRect();


            minimum_rectangles.push_back(scaled_rotated_rect);


            cv::Point2f rect_points[4];
            scaled_rotated_rect.points(rect_points);

            //list of pairs identifying the points that make up the U on the turtlebot
            std::vector<std::pair<cv::Point2f, cv::Point2f>> u_pairs;

            //draw rotatedRect
            ROS_INFO_STREAM("line values");
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
                // ROS_INFO_STREAM(avg_line_value);



                // cv::circle(dst, rect_points[j], 10, cv::Scalar(0,0,255));
            }

            if (u_pairs.size() != 3) {
                ROS_WARN_STREAM("3 lines not found from HSV images. Do something now.");
            }
            else {
                //upon observation i can just take the first and third lines. The first and third angles will be the same
                //so this gives me direction
                // float angle_line1 = std::atan((u_pairs[0].second.y - u_pairs[0].first.y)/(u_pairs[0].second.x - u_pairs[0].first.x));
                // float angle_line2 = std::atan((u_pairs[2].second.y - u_pairs[2].first.y)/(u_pairs[2].second.x - u_pairs[2].first.x));
                // cv::line( dst, u_pairs[0].first, u_pairs[0].second, color,4);


                //two of the values should be the same so we can track if this is the one we've seen
                std::vector<int> u_angles(3);
                // std::pair<cv::Point2f, cv::Point2f> direction_line;
                std::pair<cv::Point2f, cv::Point2f> front_line;
                for (std::pair<cv::Point2f, cv::Point2f>& line_points:u_pairs) {
                    // cv::line( dst, line_points.first, line_points.second, color,4);
                    //put everything in first quadrant
                    int angle_line = std::atan((line_points.second.y - line_points.first.y)/(line_points.second.x - line_points.first.x))*180.0/M_PI;
                    // if(std::find(u_angles.begin(), u_angles.end(), angle_line) != u_angles.end()) {
                    //     // direction_line = line_points;
                    //     ROS_INFO_STREAM(angle_line << " has been seen before");
                    // } 
                    // else {
                    //     ROS_INFO_STREAM(angle_line << " has not been seen before");
                    //     front_line = line_points;

                    //lets make them ints so we can compare
                    u_angles.push_back(angle_line);
                    // }

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

                //we actually just need the front line and the centoid and draw a line between the centroid and the middle of the midpoint
                cv::line( dst, front_line.first, front_line.second, color,10);
                float center_x = std::abs(front_line.first.x - front_line.second.x)/2.0;  
                float center_y = std::abs(front_line.first.y - front_line.second.y)/2.0;  

                cv::Point2f end_of_arrow(center_y, center_x);
                cv::Point2f start_of_arrow =  centroids[i];
                cv::arrowedLine(dst, start_of_arrow, end_of_arrow, color, 20);




            }

        }
    }



    // cv::Mat aligned = orb_tracker->calculate_image_alignment(mask);

    sensor_msgs::ImagePtr img_msg; // >> message to be sent
    // img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", hsv).toImageMsg();
    // image_test_pub.publish(img_msg);
    img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", dst).toImageMsg();
    // img_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", aligned).toImageMsg();

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
