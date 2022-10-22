#pragma once

#include <ros/ros.h>

#include "data_t.hpp"
#include "icp/svd_icp.hpp"

class StateEstimation
{
private:
    ros::NodeHandle node_handle_;
    ICP icp;

public:
    StateEstimation(ros::NodeHandle nh);

    void updateObstacle(Obstacle& obstacle, segment_t& segment, double dt);
};

