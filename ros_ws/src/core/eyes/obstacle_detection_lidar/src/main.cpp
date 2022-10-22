#include <ros/ros.h>
#include "obstacle_detection_lidar_node.hpp"

int main(int argc,  char* argv[])
{ 
    ROS_INFO("Node started");
    ros::init (argc, argv, "obstacle_detection_lidar");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    ScanFilter scanFilter(private_nh, nh);

    ros::spin();
    return 0;
}