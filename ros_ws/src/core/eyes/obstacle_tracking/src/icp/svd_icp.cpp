#include <limits.h>
#include <cmath>


#include <chrono>
#include <thread>


#include "visualization_msgs/MarkerArray.h"

#include "icp/svd_icp.hpp"


ICP::ICP(ros::NodeHandle &nh): node_handle_(nh) {
    rviz_pub = node_handle_.advertise<visualization_msgs::Marker>("/visualize/icp", 1);
}

void ICP::visualize(Eigen::Matrix2Xd target, Eigen::Matrix2Xd source) {
    visualization_msgs::Marker markers;

    markers.header.frame_id = "map";
    markers.header.stamp = ros::Time::now();
    markers.ns = "icp_target";
    markers.action = visualization_msgs::Marker::ADD;
    markers.pose.orientation.w = 1.0;

    markers.id = 0;
    
    markers.type = visualization_msgs::Marker::POINTS;

    // points are red
    markers.color.r = 1.0;
    markers.color.g = 0.0;
    markers.color.b = 0.0;
    markers.color.a = 1.0;

    markers.scale.x = 0.1;

    for (int i = 0; i < target.cols(); i++) {
        geometry_msgs::Point p;
        p.x = target(0, i);
        p.y = target(1, i);
        
        markers.points.push_back(p);
    }

    rviz_pub.publish(markers);

    // markers.id = 1;
    markers.ns = "icp_source";

    // points are green
    markers.color.r = 0.0;
    markers.color.g = 1.0;
    markers.color.b = 0.0;
    markers.color.a = 1.0;

    markers.points.clear();

    for (int i = 0; i < source.cols(); i++) {
        geometry_msgs::Point p;
        p.x = source(0, i);
        p.y = source(1, i);
        
        markers.points.push_back(p);
    }

    rviz_pub.publish(markers);
}

std::vector<std::pair<int, int>> ICP::get_correspondences(Eigen::Matrix2Xd target, Eigen::Matrix2Xd source) {
    std::vector<std::pair<int, int>> correspondences;

    for (unsigned int i = 0; i < source.cols(); i++) {
        double min_dist = std::numeric_limits<double>::max();
        int min_dist_index = 0;

        for (unsigned j = 0; j < target.cols(); j++) {
            double dist = std::pow(std::pow(target(0, j) - source(0, i), 2.0) + std::pow(target(1, j) - source(1, i), 2.0), 0.5);
            
            if (dist < min_dist) {
                min_dist = dist;
                min_dist_index = j;
            }
        }

        std::pair<int, int> corr_pair(i, min_dist_index);
        correspondences.push_back(corr_pair);
    }

    return correspondences;
}

Eigen::Vector2d ICP::get_center(Eigen::Matrix2Xd data, std::vector<int> exclude_indices) {
    Eigen::Matrix2Xd cleaned_data = Eigen::Matrix2Xd::Zero(2, data.cols() - exclude_indices.size());

    unsigned int j = 0;

    for (unsigned int i = 0; i < data.cols(); i++) {
        if (!exclude_indices.empty() && exclude_indices.front() == i) {
            exclude_indices.erase(exclude_indices.begin());
            j++;
        } else {
            cleaned_data(0, i - j) = data(0, i);
            cleaned_data(1, i - j) = data(1, i);
        }
    }

    Eigen::Vector2d center(cleaned_data.row(0).mean(), cleaned_data.row(1).mean());
    return center;
}

Eigen::Matrix2d ICP::get_cross_covariance(Eigen::Matrix2Xd target, Eigen::Matrix2Xd source, std::vector<std::pair<int, int>> correspondences) {
    Eigen::Matrix2d cov;

    for (unsigned int i = 0; i < correspondences.size(); i++) {
        std::pair<int, int> association = correspondences.at(i);

        Eigen::Vector2d p = source.col(association.first);
        Eigen::Vector2d q = target.col(association.second);

        // weight = kernel(p_point - q_point)
        double weight = 1.0;

        cov += weight * q * p.transpose();
    }

    return cov;
}

Eigen::Vector2d ICP::compute_svd_icp(Eigen::Matrix2Xd target, Eigen::Matrix2Xd source, unsigned int iterations) {
    Eigen::Vector2d translation;

    double angle = 20.0 / 180.0 * M_PI;
    Eigen::Matrix2d rotation_matrix;
    rotation_matrix << std::cos(angle), -std::sin(angle),
                       std::sin(angle),  std::cos(angle);

    // Eigen::Matrix2Xd s = (source.array() + 2.0).matrix();
    Eigen::Matrix2Xd s = ((rotation_matrix * source).array() + 2.0).matrix();
    // Eigen::Matrix2Xd s = source;
    Eigen::Matrix2Xd t = target;

    for (unsigned int i = 0; i < iterations; i++) {
        // std::this_thread::sleep_for(std::chrono::milliseconds(100));
        visualize(t, s);

        std::vector<int> exclude_indices;        

        Eigen::Vector2d center_target = get_center(t, exclude_indices);
        Eigen::Vector2d center_source = get_center(s, exclude_indices);

        Eigen::Matrix2Xd centered_target = t.colwise() - center_target;
        Eigen::Matrix2Xd centered_source = s.colwise() - center_source;

        std::vector<std::pair<int, int>> correspondences = get_correspondences(centered_target, centered_source);

        Eigen::Matrix2d cov = get_cross_covariance(centered_target, centered_source, correspondences);
        Eigen::JacobiSVD<Eigen::Matrix2d> svd(cov, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix2d V_T = svd.matrixV().transpose();

        //TODO do we need to transpose V?
        // Eigen::Matrix2d R = svd.matrixV() * svd.matrixU().transpose();

        Eigen::Matrix2d R = svd.matrixU() * svd.matrixV().transpose();

        if (R.determinant() < 0)
        {
            R.col(1) *= -1.;
        }

        translation = center_target - R * center_source;

        s = (R * s).colwise() + translation;
    }

    return translation;
}
