#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"

#include <iostream>
#include <stdio.h>


#include <string>
#include <vector>
#include <algorithm>
#include <numeric>


#include "turtlebot.hpp"

struct Vec4d {
    double x;
    double y;
    double z;
    double w;
};


tf::Quaternion inverse_sign_quaternion(tf::Quaternion& q);
bool are_quats_close(tf::Quaternion& q1, tf::Quaternion& q2);
tf::Quaternion average_quaternion(Vec4d& cumulative, tf::Quaternion& newRotation, tf::Quaternion& firstRotation, int addAmount);




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

    geometry_msgs::PoseStamped pose_filtered;
    camera_frame_history.push_back(pose);

    if(filter_poses(pose_filtered)) {
        camera_frame_odom_pub.publish(pose_filtered);
    }

}

bool Turtlebot::filter_poses(geometry_msgs::PoseStamped& filtered_pose) {

    double yaw_average;
    double x_average;
    double y_average;

    Vec4d vec = {0.0, 0.0, 0.0, 0.0};

    if (camera_frame_history.size() < 2) {
        return false;
    }
    //get first quat for averaging. This may be problem if the first one is bad
    tf::Quaternion quat_first;
    tf::quaternionMsgToTF(camera_frame_history[0].pose.orientation, quat_first);

    ROS_INFO_STREAM("averaging " << camera_frame_history.size() << " poses");
    //get averages
    for(auto& pose :camera_frame_history) {
        tf::Quaternion quat;
        tf::quaternionMsgToTF(pose.pose.orientation, quat);

        //if averaging quats dont work we can just average yaw
        // double roll, pitch, yaw;
        // tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

        average_quaternion(vec, quat, quat_first, camera_frame_history.size());

        // yaw_average += yaw;
        x_average += pose.pose.position.x;
        y_average += pose.pose.position.y;

    }

    //we have now averaged the quaternions so this should now be the average yaw
    tf::Quaternion quat_cumulative(vec.x, vec.y, vec.z, vec.z);
    double roll, pitch, average_yaw;
    tf::Matrix3x3(quat_cumulative).getRPY(roll, pitch, average_yaw);

    x_average/=camera_frame_history.size();
    y_average/=camera_frame_history.size();

    ROS_INFO_STREAM("average " << x_average << "  "  << y_average << " " << average_yaw);

    return true;

}

const geometry_msgs::PoseStamped& Turtlebot::get_latest_camera_pose() const {
    return camera_frame_history[camera_frame_history.size() - 1];
}



//some code to help with Quaternions
//http://wiki.unity3d.com/index.php/Averaging_Quaternions_and_Vectors

tf::Quaternion average_quaternion(Vec4d& cumulative, tf::Quaternion& newRotation, tf::Quaternion& firstRotation, int addAmount){
 
	float w = 0.0f;
	float x = 0.0f;
	float y = 0.0f;
	float z = 0.0f;
 
	//Before we add the new rotation to the average (mean), we have to check whether the quaternion has to be inverted. Because
	//q and -q are the same rotation, but cannot be averaged, we have to make sure they are all the same.
	if(!are_quats_close(newRotation, firstRotation)){
 
		newRotation = inverse_sign_quaternion(newRotation);	
	}
 
	//Average the values
	float addDet = 1.0f/(float)addAmount;
	cumulative.w += newRotation.w();
	w = cumulative.w * addDet;
	cumulative.x += newRotation.x();
	x = cumulative.x * addDet;
	cumulative.y += newRotation.y();
	y = cumulative.y * addDet;
	cumulative.z += newRotation.z();
	z = cumulative.z * addDet;		
 
	//note: if speed is an issue, you can skip the normalization step
	// return NormalizeQuaternion(x, y, z, w);
    return tf::Quaternion(x, y, w, z);

}

//Changes the sign of the quaternion components. This is not the same as the inverse.
tf::Quaternion inverse_sign_quaternion(tf::Quaternion& q){
 
	return tf::Quaternion(-q.x(), -q.y(), -q.z(), -q.w());
}

bool are_quats_close(tf::Quaternion& q1, tf::Quaternion& q2){
 
	float dot = q1.dot(q2);
 
	if(dot < 0.0f){
 
		return false;					
	}
 
	else{
 
		return true;
	}
}
