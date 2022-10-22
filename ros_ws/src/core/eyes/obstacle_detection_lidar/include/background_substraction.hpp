#pragma once

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <tf/transform_listener.h>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "grid_map.hpp"

#include "data.hpp"

// using namespace SCAN_OBSTACLE_FILTER;

/**
 * @brief This class takes a pointcloud and removes all points that can be 
 * mapped to an occupied grid on the OccupancyMap. The incoming pointcloud has
 * to be from the map frame.
 */

class OccupancyBackgroundSubtraction
{
    
public:
    OccupancyBackgroundSubtraction(ros::NodeHandle &nh);

    std::vector<beam_t> backgroundSubstraction(
        map_t* current_map,
        std::vector<beam_t> laser_beams
    );

    sensor_msgs::PointCloud2 getCloud();

private:
    ros::NodeHandle node_handle_;

    sensor_msgs::PointCloud2 cloud_;

    tf::TransformListener tf_listener;
    ros::Publisher visu_publisher_;

    double threshold_;
    std::string map_frame_;
    std::string car_frame_;
};

