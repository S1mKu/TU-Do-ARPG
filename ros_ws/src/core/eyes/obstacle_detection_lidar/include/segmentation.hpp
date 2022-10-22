#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>

// PCL specific includes
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "eyes_msgs/Segment.h"
#include "eyes_msgs/SegmentList.h"

#include "data.hpp"

class AdaptiveBreakpointDetection {

public:

    AdaptiveBreakpointDetection(ros::NodeHandle& nh);

    double compute_threshold(beam_t b1, beam_t b2);
    eyes_msgs::SegmentList computeSegments(std::vector<beam_t> laser_beams);

private:

    ros::NodeHandle node_handle_;

    void loadParameter();

    double lambda_;
    double short_range_increase_basis_;
    double short_range_increase_exp_bias_;
    double lidar_standard_deviation_;

    int max_breakpoints_count = 0;

    int cur_num_obstacles = 0;

    ros::Publisher visu_publisher_;

};
