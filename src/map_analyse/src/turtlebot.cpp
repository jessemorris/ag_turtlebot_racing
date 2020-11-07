#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>

#include <iostream>
#include <stdio.h>


#include <string>


#include "turtlebot.hpp"


Turtlebot::Turtlebot(ros::NodeHandle& _nh, const geometry_msgs::PoseStamped& _inital_pose, int _history_size):
    nh(_nh),
    inital_pose_camera(_inital_pose),
    history_size(_history_size) {

        camera_frame_odom_pub = nh.advertise<geometry_msgs::PoseStamped>("turtlebot_pose", 1000);
    }

void Turtlebot::update_pose_camera(geometry_msgs::PoseStamped& pose) {
    if (camera_frame_history.size() >= history_size) {
        //remove the first element
        camera_frame_history.erase(camera_frame_history.begin());
    }
    camera_frame_history.push_back(pose);
    camera_frame_odom_pub.publish(pose);

}

const geometry_msgs::PoseStamped& Turtlebot::get_latest_camera_pose() const {
    return camera_frame_history[camera_frame_history.size() - 1];
}
