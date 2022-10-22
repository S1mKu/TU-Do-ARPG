#include "background_substraction.hpp"

#include "visualization_msgs/MarkerArray.h"

#include <math.h>

OccupancyBackgroundSubtraction::OccupancyBackgroundSubtraction(ros::NodeHandle &nh):
    node_handle_(nh) {
    if (!node_handle_.param<double>("algorithms/bgs/threshold", threshold_, 1.0))
    {
        ROS_WARN_STREAM("Did not load bgs theshold.");
    }

    if (!node_handle_.param<std::string>("tf/frames/map_frame", map_frame_, "/map"))
    {
        ROS_WARN_STREAM("Did not load global frame name.");
    }

    if (!node_handle_.param<std::string>("tf/frames/car_frame", car_frame_, "/base_link"))
    {
        ROS_WARN_STREAM("Did not load base_link frame name.");
    }

    std::string ns = ros::this_node::getNamespace();
    std::string frame_prefix = "opp_racecar";

    if (ns == "/a1")
    {
        frame_prefix = "ego_racecar";
    }

    car_frame_ = frame_prefix + car_frame_;

    visu_publisher_ = node_handle_.advertise<visualization_msgs::Marker>("bs", 1);
}

sensor_msgs::PointCloud2 OccupancyBackgroundSubtraction::getCloud() {
    return cloud_;
}

std::vector<beam_t> OccupancyBackgroundSubtraction::backgroundSubstraction(
    map_t* current_map,
    std::vector<beam_t> laser_beams
) {
    std::vector<beam_t> filtered_beams;
    
    for (beam_t beam : laser_beams)
    {
        map_cell_t* cell = map_get_cell(current_map, beam.endpoint_x, beam.endpoint_y);

        if (cell != NULL)
        {
            if (cell->occ_state == -1 && cell->occ_dist_inner >= 4.0 && cell->occ_dist_outer >= 4.0)
            {
                filtered_beams.push_back(beam);
            }
        }
        else
        {
            ROS_WARN_STREAM("BS - INVALID BEAM: " << beam.endpoint_x << " " << beam.endpoint_y);
        }
    }

    return filtered_beams;
}
