#pragma once

#include <vector>
#include <Eigen/Dense>
#include <boost/circular_buffer.hpp>
#include <ros/ros.h>

typedef std::pair<double, double> Point;
typedef std::vector<Point> PointList;


enum ObstacleType
{
    UNKNOWN,
    STATIC,
    DYNAMIC
};


typedef struct Association
{
    int obstacle_index;
    int segment_index;
    double distance;
    bool obstacle_unassociated = false;
} association_t;


typedef struct SegmentWrapper
{
    PointList points;
    bool invalidated = false;
} segment_t;


class Obstacle
{

public:
    ObstacleType type;

    ros::Time timestamp;

    bool invalidated = false;

    boost::circular_buffer<Point> position_backlog;
    boost::circular_buffer<segment_t> segment_backlog;
    boost::circular_buffer<double> theta_backlog;

    double theta;
    double velocity;

    int leftover_index = 0;

    Obstacle(segment_t segment, Point position, double theta, int len_backlog, ObstacleType type);
    
};
