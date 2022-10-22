#include "data_t.hpp"

#include <ros/ros.h>

Obstacle::Obstacle(segment_t segment, Point position, double theta, int len_backlog, ObstacleType type)
{
    type = type;
    position_backlog = boost::circular_buffer<Point>(len_backlog);
    position_backlog.push_back(position);
    theta_backlog = boost::circular_buffer<double>(len_backlog);
    theta_backlog.push_back(theta);
    segment_backlog = boost::circular_buffer<segment_t>(2);
    segment_backlog.push_back(segment);
    theta = 0.0;
    velocity = 0.0;

    timestamp = ros::Time::now();
}
