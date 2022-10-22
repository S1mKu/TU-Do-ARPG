#pragma once

#include <Eigen/Dense>
#include <vector>

#include <ros/ros.h>

class ICP {

private:
    ros::NodeHandle node_handle_;
    ros::Publisher rviz_pub;

public:
    ICP(ros::NodeHandle& nh);

    void visualize(Eigen::Matrix2Xd target, Eigen::Matrix2Xd source);

    std::vector<std::pair<int, int>> get_correspondences(Eigen::Matrix2Xd target, Eigen::Matrix2Xd source);
    Eigen::Vector2d get_center(Eigen::Matrix2Xd data, std::vector<int> exclude_indices);
    Eigen::Matrix2d get_cross_covariance(Eigen::Matrix2Xd target, Eigen::Matrix2Xd source, std::vector<std::pair<int, int>> correspondences);
    Eigen::Vector2d compute_svd_icp(Eigen::Matrix2Xd target, Eigen::Matrix2Xd source, unsigned int iterations);

}; 

