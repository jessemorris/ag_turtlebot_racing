#include <ros/ros.h>


#include "map_analyse.hpp"



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