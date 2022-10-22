#pragma once

#include <vector>

typedef std::pair<double, double> Point;
typedef std::vector<Point> PointList;


typedef struct Beam {
    double endpoint_x;
    double endpoint_y;
    double range;
    double theta;
    unsigned int index; // position/inidex in the original lidar scan
    int cluster_id; // beams falling into neighboring cells in the occupancy grid map obtain the same cluster_id
} beam_t;
