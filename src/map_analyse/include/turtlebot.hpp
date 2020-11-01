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

class Turtlebot {


    public:
        Turtlebot(const geometry_msgs::PoseStamped& _inital_pose, int _history_size = 5);
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
        //how many poses to keep in memory
        int history_size;

        std::vector<geometry_msgs::PoseStamped> camera_frame_history;
        const geometry_msgs::PoseStamped& inital_pose_camera;
        //will eventually need some ros stuff here i think



};



#endif