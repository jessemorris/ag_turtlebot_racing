#ifndef AR_TURTLEBOT_TURTLEBOT
#define AR_TURTLEBOT_TURTLEBOT


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

#include <vector>
#include <string>

struct Vec4d {
    double x;
    double y;
    double z;
    double w;
};


class Turtlebot {


    public:
        Turtlebot(ros::NodeHandle& _nh, const geometry_msgs::PoseStamped& _inital_pose, int _history_size = 20);
        ~Turtlebot() {}

        /**
         *   Updates the current position of the robot calculated from the camera frame
         *   eg. x,y etc will be in frame coordinates
         */
        void update_pose_camera(geometry_msgs::PoseStamped& pose);

        /**
         * Returns the latest pose from the camera frame
         */
        const geometry_msgs::PoseStamped& get_latest_camera_pose() const;

    private:
        ros::NodeHandle nh;

        ros::Publisher camera_frame_odom_pub;

        //how many poses to keep in memory
        int history_size;

        std::vector<geometry_msgs::PoseStamped> camera_frame_history;
        const geometry_msgs::PoseStamped& inital_pose_camera;
        //will eventually need some ros stuff here i think

        /**
         * finds the standard deviation of the values in the pose array. Specifically the
         */
        bool compute_average_pose(double& x_average, double& y_average, Vec4d& quat_average);
        // bool compute_standard_deviation(double x_average, double y_average, double&yaw_average)
        std::unique_ptr<geometry_msgs::PoseStamped> filter_poses(double x_average, double y_average, const Vec4d& vec);



};



#endif
