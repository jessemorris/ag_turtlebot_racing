#include <ros/ros.h>


#include "map_analyse.hpp"



int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_analyse");
    ros::NodeHandle n;

    MapAnalyse map_analyse(n);
    ros::spin();
}
