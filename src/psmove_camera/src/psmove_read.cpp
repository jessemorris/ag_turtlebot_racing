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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "psmove_camera");



    ros::NodeHandle n;

    std::string output_video_topic;
    n.getParam("psmove_camera/image_topic", output_video_topic);

    int device_id;
    n.getParam("psmove_camera/cam_source", device_id);

    // int cam_source;
    // n.getParam("psmove_camera/cam_source", cam_source);

    // std::cout << "\ntesting:" << cam_source << "\n" << std::endl;
    // device_id = cam_source;


    image_transport::ImageTransport it(n);
    image_transport::Publisher pub_img = it.advertise(output_video_topic, 20);

    cv::VideoCapture cap;

    if (!cap.open(device_id)) {
        ROS_ERROR("Camera failed to open");
        return -1;
    }
    //cap.set(cv::CAP_PROP_XI_AUTO_WB, 0);
    // cap.set(cv::CAP_PROP_AUTO_WB, 0);


    cv::Mat image;
    sensor_msgs::ImagePtr img_msg;

    sensor_msgs::CameraInfo camera_info;
    // ros::Rate r(10);

    while (ros::ok()) {
        cap.read(image);
        ROS_INFO_STREAM(image.rows << " " << image.cols);

        if (image.empty()) {
            ROS_WARN("Frame was empty");
        }
        else {
            img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
            pub_img.publish(img_msg);
        }
        // r.sleep();
        ros::spinOnce();

    }
    return 0;
}
