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

// handler.param<std::string>("/realtime_vdo_slam/topic_prefix", topic_prefix, "/gmsl/");

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
        // nh.param<std::string>("/realtime_vdo_slam/topic_prefix", topic_prefix, "/gmsl/");

        //min range
        nh.getParam("/object_placement/blue_threshold/min_threshold_1", blue_min_threshold_1);
        nh.getParam("/object_placement/blue_threshold/max_threshold_1", blue_max_threshold_1);

        //higher range
        nh.getParam("/object_placement/blue_threshold/min_threshold_2", blue_min_threshold_2);
        nh.getParam("/object_placement/blue_threshold/max_threshold_2", blue_max_threshold_2);
        // nh.param<std::vector<int>>("/map_analyse/blue_threshold/min_threshold", blue_min_threshold);
        // nh.param<std::vector<int>>("/map_analyse/blue_threshold/max_threshold", blue_max_threshold);

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

    // l_img = cv2.imread("larger_image.jpg")
    // int x_offset= 50;
    // int y_offset=50;
    // Rect ROI (x, y, width, height);
    Rect ROI (50, 50, 200, 200);

    // Mat channel[4];
    vector<Mat> channel;

    split(banana_image, channel);

    // Mat alpha = channel[3];// / 255.0


    // //
    // //
    // // // Convert Mat to float data type
    // banana_image_colour.convertTo(banana_image_colour, CV_32FC3);
    //
    // // The actual splitting.
    //
    //
    // image.convertTo(image, CV_32FC3);
    //
    // // // Normalize the alpha mask to keep intensity between 0 and 1
    // alpha.convertTo(alpha, CV_32FC3, 1.0/255); //
    // // // alpha.convertTo(alpha, CV_32FC3); //
    // //
    // // //
    // // // Storage for output image
    //
    // Mat ouImage = Mat::zeros(banana_image_colour.size(), banana_image_colour.type());
    // // Multiply the foreground with the alpha matte
    // multiply(alpha, banana_image_colour, banana_image_colour);
    //
    // // Multiply the background with ( 1 - alpha )
    // multiply(Scalar::all(1.0)-alpha, image, image);
    //
    // // Add the masked foreground and background.
    // add(banana_image, image, ouImage);




    // Mat mask;
    // Mat layers[4];
    //
    // split(transp, layers); // seperate channels
    Mat rgb[3] = { channel[0],channel[1],channel[2] };
    Mat mask_layer = channel[3]; // png's alpha channel used as mask

    merge(rgb, 3, banana_image);  // put together the RGB channels, now transp insn't transparent
    int yPos = 10;
    int xPos = 10;
    banana_image.copyTo(image.rowRange(yPos, yPos + banana_image.rows).colRange(xPos, xPos + banana_image.cols), mask_layer);



    // l_img[y_offset:y_offset+s_img.shape[0], x_offset:x_offset+s_img.shape[1]] = s_img


    // resize(banana_image, banana_image, Size(ROI.height, ROI.width));
    // banana_image.copyTo( image(ROI) );




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


    img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

    // img_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", alpha).toImageMsg();

    // img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", ouImage).toImageMsg();


    // img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", banana_image).toImageMsg();

    image_test_pub.publish(img_msg);
}
//
// void ObjectPlacement::worldToCamera(const sensor_msgs::ImageConstPtr& msg) {
//
//     // p = point in world frame [x,y,z,0]
//     double p[4] = {x,y,z,0};
//
//
//     // R = camera rotation matrix
//     // T = camera translation matrix
//
//
//
//     //K = intrinsic matrix
//     double K[3][4] = {{519.566467 0.000000 313.837735 0.000000},
//     {0.000000 522.066406 248.386084 0.000000},
//     {0.000000 0.000000 1.000000 0.000000}};
//
// 
//     //extrinsic_matrix = [R', T'; 0,0,0,1]
//     double extrinsic_matrix[4][4] = {{R,T},{0,0,0,1}};
//
//
//     //camera_frame = K*extrinsic_matrix*p'
//     double camera_frame
//
//     // these are the undistored points
//     //u = camera_frame_2(1)/camera_frame_2(3);
//     //v = camera_frame_2(2)/camera_frame_2(3);
//
//      //u0 = principal_point x
//      //v0 = principal_point y
//
//      //fx = focal length x
//      //fy = focal length y
//
//      // x = (u-u0)/fx;
//      // y = (v-v0)/fy;
//
//      //r = sqrt(x^2+y^2);
//
//      //k1, k2, k3 is radial distortion
//
//      //k4,k5 is tangential distortion
//
//      //xd = x*(1+k1*r^2 +k2*r^4 + k3*r^6) + 2*k4*x*y+k5*(r^2+2*x^2);
//      //yd = y*(1+k1*r^2 +k2*r^4 + k3*r^6)+k4*(r^2+2*y^2)+2*k5*x*y;
//
//      // distorted pixel coordinates
//      //ud = fx*xd + u0;
//      // vd= fy*yd + v0;
//
//
// }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_placement");
    ros::NodeHandle n;

    ObjectPlacement object_placement(n);
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
