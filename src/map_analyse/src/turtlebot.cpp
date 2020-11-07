#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>

#include <iostream>
#include <stdio.h>


#include <string>


#include "turtlebot.hpp"


Turtlebot::Turtlebot(const geometry_msgs::PoseStamped& _inital_pose, int _history_size):
    inital_pose_camera(_inital_pose),
    history_size(_history_size) {}

void Turtlebot::update_pose_camera(geometry_msgs::PoseStamped& pose) {
    if (camera_frame_history.size() >= history_size) {
        //remove the first element
        camera_frame_history.erase(camera_frame_history.begin());
    }
    camera_frame_history.push_back(pose);

}

const geometry_msgs::PoseStamped& Turtlebot::get_latest_camera_pose() const {
    return camera_frame_history[camera_frame_history.size() - 1];
}
