#include <ros/ros.h>
#include <image_transport/image_transport.h>


#include "virtual_object.hpp"



int  main (int argc, char** argv){

	ros::init(argc, argv, "mesh_visualiser");
	ros::NodeHandle nh;

    VirtualObject obj(nh, "bunny", 5, 5);
	VirtualObject ob1(nh, "dragon", 7, 7, ".ply");

    while(ros::ok()) {
        ros::spinOnce();

    }

	// image_transport::ImageTransport it(nh);
	// image_transport::Publisher pub_img = it.advertise("mesh_vis", 20);
	// sensor_msgs::ImagePtr img_msg;
}