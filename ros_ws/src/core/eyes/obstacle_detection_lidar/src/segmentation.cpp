#include <cmath>
#include <cfloat>
#include <vector>
#include <limits>
#include <algorithm>

#include <geometry_msgs/PointStamped.h>

#include "visualization_msgs/MarkerArray.h"

#include "segmentation.hpp"

typedef std::pair<double, double> Point;
typedef std::vector<Point> PointList;

AdaptiveBreakpointDetection::AdaptiveBreakpointDetection(ros::NodeHandle& nh):
    node_handle_(nh) {
    loadParameter();

    visu_publisher_ = node_handle_.advertise<visualization_msgs::Marker>("abd", 1);
}

void AdaptiveBreakpointDetection::loadParameter() {
    if (!node_handle_.param<double>("algorithms/abd/lambda", lambda_, 5.0))
    {
        ROS_WARN_STREAM("Did not load abp lambda.");
    }

    // convert lambda_ from degree to rad
    lambda_ = lambda_ / 180.0 * M_PI;

    if (!node_handle_.param<double>("algorithms/abd/short_range_increase_basis", short_range_increase_basis_, 2.0))
    {
        ROS_WARN_STREAM("Did not load abp short_range_increase_basis.");
    }

    if (!node_handle_.param<double>("algorithms/abd/short_range_increase_exp_bias", short_range_increase_exp_bias_, 4.0))
    {
        ROS_WARN_STREAM("Did not load abp short_range_increase_exp_bias.");
    }

    if (!node_handle_.param<double>("algorithms/abd/lidar_standard_deviation", lidar_standard_deviation_, 0.0))
    {
        ROS_WARN_STREAM("Did not load abp lidar_standard_deviation.");
    }
}

double AdaptiveBreakpointDetection::compute_threshold(beam_t b1, beam_t b2)
{
    // use this code to determine the right parameters for [1.0 / std::pow(A, range + B)]
    /*
    import matplotlib.pyplot as plt
    import numpy as np
    import math

    margin = 4.0
    exp = 2computeSegments
    ranges = np.arange(-1.0, 3.0, 0.01)

    # ads = np.array(map(lambda r : 1.0/ (r + margin)**exp, ranges))
    ads = np.array(map(lambda r : 1.0/ (exp**(r + margin)), ranges))

    dtheta = math.radians(0.25)
    l = math.radians(5.0)

    thresholdFctn = lambda r : r * math.sin(dtheta) / math.sin(l - dtheta)
    results = np.array(map(thresholdFctn, ranges))

    fig, ax = plt.subplots()
    ax.plot(ranges, results)
    results = results + ads
    ax.plot(ranges, results)
    plt.show()
    */

    double range = std::min(b1.range, b2.range);
    double dtheta = std::abs(b1.theta - b2.theta);

    // this might be an aternative adaptive_threshold realization
    // double adaptive_threshold = range * std::sin(dtheta) / std::sin(l - dtheta) + 3 * s + 1.0 / std::pow(range + short_range_increase_basis_, short_range_increase_exp_bias_);

    double low_range_bias = 1.0 / std::pow(short_range_increase_basis_, range + short_range_increase_exp_bias_);
    double noise_bias = 3 * lidar_standard_deviation_;
    double adaptive_threshold = range * std::sin(dtheta) / std::sin(lambda_ - dtheta) + noise_bias + low_range_bias;

    return adaptive_threshold;
}

/**
 * @brief Segmentation Algorithms following the clustering
 *  from the datmo project
 *  see: https://github.com/kostaskonkk/datmo
 *
 */
eyes_msgs::SegmentList AdaptiveBreakpointDetection::computeSegments(std::vector<beam_t> laser_beams) {
    eyes_msgs::SegmentList obstacles;

    const int c_points = laser_beams.size();

    if (c_points == 0)
    {
        ROS_WARN_STREAM("NO CLUSTER FOUND!");
        return obstacles;
    }

    eyes_msgs::Segment segment;

    int num_regular_breakpoints = 0;
    int num_neighborhood_breakpoints = 0;
    int num_breakpoints = 0;
    int inf_counter = 0;
    int point_counter = 0;

    visualization_msgs::Marker markers;
    markers.header.frame_id = "map";
    markers.header.stamp = ros::Time::now();
    markers.ns = "breakpoints";
    markers.action = visualization_msgs::Marker::ADD;
    markers.pose.orientation.w = 1.0;
    markers.id = 0;
    markers.type = visualization_msgs::Marker::LINE_STRIP;
    markers.color.r = 1.0;
    markers.color.g = 1.0;
    markers.color.b = 0.0;
    markers.color.a = 1.0;
    markers.scale.x = 0.1;

    for (int i = 0; i < c_points - 1 /* substract one since we look at points [i] and [i+1] */; i++)
    {
        // get the two points to look at in this iteration
        beam_t beam_i = laser_beams.at(i);
        beam_t beam_iplus1 = laser_beams.at(i + 1);

        // skip this beam when its range is out of bounds
        if (beam_i.range == std::numeric_limits<double>::infinity())
        {
            inf_counter++;
            continue;
        }

        // current point is always in a segment
        // check for breakpoint/new segment between current and next point
        geometry_msgs::Point p;
        p.x = beam_i.endpoint_x;
        p.y = beam_i.endpoint_y;
        segment.points.push_back(p);

        point_counter++;

        if (beam_i.cluster_id != beam_iplus1.cluster_id) // beams of neighboring cells are not considered for adaptive breakpoint detection
        {
            bool is_breakpoint = true;
            int index_threshold = std::max(2, static_cast<int>(std::floor(8.0 - 2.5 * std::max(beam_i.range, beam_iplus1.range))));

            if (beam_iplus1.index - beam_i.index <= index_threshold) // only points with an angle of LaserScan.angle_increment apart are considered for adaptive breakpoint detection
            {
                double euclidean_dist = std::sqrt(std::pow(beam_i.endpoint_x - beam_iplus1.endpoint_x, 2) + std::pow(beam_i.endpoint_y - beam_iplus1.endpoint_y, 2));

                if (euclidean_dist < 0.06) // only check points that are spacially separated
                {
                    is_breakpoint = false;
                }
                else
                {
                    double adaptive_threshold = compute_threshold(beam_i, beam_iplus1);

                    if (adaptive_threshold >= euclidean_dist)
                    {
                        is_breakpoint = false;
                    }
                }
            }

            if (is_breakpoint)
            {
                num_regular_breakpoints++;

                markers.id = num_breakpoints++;
                geometry_msgs::Point p1;
                p1.x = beam_i.endpoint_x;
                p1.y = beam_i.endpoint_y;

                geometry_msgs::Point p2;
                p2.x = segment.points.front().x;
                p2.y = segment.points.front().y;

                markers.points.push_back(p1);
                markers.points.push_back(p2);
                visu_publisher_.publish(markers);
                markers.points.clear();

                // BREAKPOINT - switch to a new segment
                obstacles.segments.push_back(segment);
                segment = eyes_msgs::Segment();
            }
        }
    }

    // put the last point into the current segment
    beam_t last_beam = laser_beams.back();

    geometry_msgs::Point p;
    p.x = last_beam.endpoint_x;
    p.y = last_beam.endpoint_y;

    segment.points.push_back(p);
    obstacles.segments.push_back(segment);

    int num_obstacles = obstacles.segments.size();

    cur_num_obstacles = obstacles.segments.size();
    return obstacles;
}

