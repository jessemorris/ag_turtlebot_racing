#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <tf2/convert.h>
#include "tf/transform_datatypes.h"
#include <geometry_msgs/PoseStamped.h>
#include "geometry_msgs/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2/transform_datatypes.h>

#include <iostream>
#include <stdio.h>
#include <memory>


#include <string>
#include <vector>
#include <algorithm>
#include <numeric>


#include "turtlebot.hpp"


tf2::Quaternion inverse_sign_quaternion(tf2::Quaternion& q);
bool are_quats_close(tf2::Quaternion& q1, tf2::Quaternion& q2);
tf2::Quaternion average_quaternion(Vec4d& cumulative, tf2::Quaternion& newRotation, tf2::Quaternion& firstRotation, int addAmount);




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

    double x_average, y_average;
    Vec4d vec = {0.0, 0.0, 0.0, 0.0};
    compute_average_pose(x_average, y_average, vec);

    geometry_msgs::PoseStamped pose_filtered;
    camera_frame_history.push_back(pose);

    std::unique_ptr<geometry_msgs::PoseStamped> filtered_pose = filter_poses(x_average, y_average, vec);

    if(filtered_pose) {
        camera_frame_odom_pub.publish(*filtered_pose);
    }

}

bool Turtlebot::compute_average_pose(double& x_average, double& y_average, Vec4d& quat_average) {



    if (camera_frame_history.size() < 2) {
        return false;
    }
    //get first quat for averaging. This may be problem if the first one is bad
    tf2::Quaternion quat_first;
    // tf2::quaternionMsgToTF(camera_frame_history[0].pose.orientation, quat_first);
    tf2::convert(camera_frame_history[0].pose.orientation, quat_first);

    ROS_INFO_STREAM("averaging " << camera_frame_history.size() << " poses");
    //get averages
    for(auto& pose :camera_frame_history) {
        tf2::Quaternion quat;
        // tf2::quaternionMsgToTF(pose.pose.orientation, quat);
        tf2::convert(pose.pose.orientation, quat);

        //if averaging quats dont work we can just average yaw
        // double roll, pitch, yaw;
        // tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

        average_quaternion(quat_average, quat, quat_first, camera_frame_history.size());

        // yaw_average += yaw;
        x_average += pose.pose.position.x;
        y_average += pose.pose.position.y;

    }

    //we have now averaged the quaternions so this should now be the average yaw
    // tf2::Quaternion quat_cumulative(vec.x, vec.y, vec.z, vec.w);
    // double roll, pitch;
    // tf2::Matrix3x3(quat_cumulative).getRPY(roll, pitch, yaw_average);

    x_average/=camera_frame_history.size();
    y_average/=camera_frame_history.size();


    //
    return true;

    // std::unique_ptr<geometry_msgs::PoseStamped> average_pose = std::make_unique<geometry_msgs::PoseStamped>();
    //
    // geometry_msgs::Quaternion ori;
    // tf2::convert(quat_cumulative, ori);
    //
    // pose_stamped->pose.orientation = ori;
    //
    //
    // pose_stamped->pose.position.x = x_average;
    // pose_stamped->pose.position.y = y_average;
    // pose_stamped->pose.position.z = 0;
    //
    // std_msgs::Header header;
    // header.frame_id = "/map";
    // header.stamp = ros::Time::now();
    //
    // pose_stamped->header = header;
    //
    //
    // return pose_stamped;

}

std::unique_ptr<geometry_msgs::PoseStamped> Turtlebot::filter_poses(double x_average, double y_average, const Vec4d& vec) {

    tf2::Quaternion quat_cumulative(vec.x, vec.y, vec.z, vec.w);
    double roll, pitch, yaw_average;
    tf2::Matrix3x3(quat_cumulative).getRPY(roll, pitch, yaw_average);

    ROS_INFO_STREAM("average " << x_average << "  "  << y_average << " " << yaw_average);

    std::unique_ptr<geometry_msgs::PoseStamped> average_pose = std::make_unique<geometry_msgs::PoseStamped>();


    geometry_msgs::Quaternion ori;
    tf2::convert(quat_cumulative, ori);

    average_pose->pose.orientation = ori;


    average_pose->pose.position.x = x_average;
    average_pose->pose.position.y = y_average;
    average_pose->pose.position.z = 0;

    std_msgs::Header header;
    header.frame_id = "/turtlebot_image_frame";
    header.stamp = ros::Time::now();

    average_pose->header = header;


    return average_pose;

}



const geometry_msgs::PoseStamped& Turtlebot::get_latest_camera_pose() const {
    return camera_frame_history[camera_frame_history.size() - 1];
}



//some code to help with Quaternions
//http://wiki.unity3d.com/index.php/Averaging_Quaternions_and_Vectors

tf2::Quaternion average_quaternion(Vec4d& cumulative, tf2::Quaternion& newRotation, tf2::Quaternion& firstRotation, int addAmount){

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
    return tf2::Quaternion(x, y, z, w);

}

//Changes the sign of the quaternion components. This is not the same as the inverse.
tf2::Quaternion inverse_sign_quaternion(tf2::Quaternion& q){

	return tf2::Quaternion(-q.x(), -q.y(), -q.z(), -q.w());
}

bool are_quats_close(tf2::Quaternion& q1, tf2::Quaternion& q2){

	float dot = q1.dot(q2);

	if(dot < 0.0f){

		return false;
	}

	else{

		return true;
	}
}
