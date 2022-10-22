#include <memory>

// roscpp
#include <ros/ros.h>

#include "obstacle_tracking.hpp"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "obstacle_tracking");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    ObstacleTracking detection(private_nh, nh);

    ros::spin();
    return EXIT_SUCCESS;
}
