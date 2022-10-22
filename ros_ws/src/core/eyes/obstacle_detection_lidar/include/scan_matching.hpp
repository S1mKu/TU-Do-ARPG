#pragma once

#include <Eigen/Dense>
#include <vector>

#include "stddef.h"

#include <ros/ros.h>

#include "geometry_msgs/Polygon.h"

#include "grid_map.hpp"
#include "data.hpp"

class ICP {

private:
    ros::NodeHandle node_handle_;
    ros::Publisher rviz_pub;
    ros::Publisher rviz_pub_arr;

public:
    ICP(ros::NodeHandle& nh);

    void visualize(Eigen::Matrix2Xd target, Eigen::Matrix2Xd source);

    std::vector<std::pair<int, int>> get_correspondences(Eigen::Matrix2Xd target, Eigen::Matrix2Xd source);
    Eigen::Vector2d get_center(Eigen::Matrix2Xd data, std::vector<int> exclude_indices);
    double compute_error(Eigen::Matrix2Xd source, Eigen::Matrix2Xd target);
    bool is_inner_beam(beam_t beam, geometry_msgs::Polygon *centerline);
    Eigen::Matrix2d get_cross_covariance(Eigen::Matrix2Xd target, Eigen::Matrix2Xd source, std::vector<std::pair<int, int>> correspondences);
    std::vector<beam_t> compute_svd_icp(map_t *map, std::vector<beam_t> beams_orig, unsigned int iterations, geometry_msgs::Polygon *centerline);
    std::vector<beam_t> compute_ls_icp(map_t *map, std::vector<beam_t> beams_orig, unsigned int iterations, geometry_msgs::Polygon *centerline);

}; 

