#include "obstacle_detection_lidar_visualization.hpp"

#include <iostream>
#include <chrono>
#include <thread>

ScanFilterVisualization::ScanFilterVisualization(ros::NodeHandle& nh):
    node_handle_(nh) {

    loadParameter();
    setupPublisher();
} 

void ScanFilterVisualization::loadParameter() {
    std::string ns = ros::this_node::getNamespace();
    std::string frame_prefix = "opp_racecar";

    if (ns == "/a1")
    {
        frame_prefix = "ego_racecar";
    }

    if (!node_handle_.param<std::string>("tf/frames/laser_frame", LASER_FRAME_, "")) 
        ROS_WARN_STREAM("Did not load baselink");   
    if (!node_handle_.param<std::string>("tf/frames/map_frame", MAP_FRAME_, ""))
        ROS_WARN_STREAM("Did not load map frame.");

    LASER_FRAME_ = frame_prefix + LASER_FRAME_;
}

void ScanFilterVisualization::setupPublisher() {
    visu_segment_publisher_ = node_handle_.advertise<visualization_msgs::Marker>("segments", 1);
}

void ScanFilterVisualization::visualizeScan(sensor_msgs::LaserScan scan)
{
    visualization_msgs::Marker markers;

    markers.header.frame_id = "ego_racecar/base_link";
    markers.header.stamp = ros::Time::now();
    markers.ns = "unprocessed_scan";
    markers.action = visualization_msgs::Marker::ADD;
    markers.pose.orientation.w = 1.0;

    markers.id = 0;
    
    markers.type = visualization_msgs::Marker::POINTS;

    // points are blue
    markers.color.r = 0.0;
    markers.color.g = 0.0;
    markers.color.b = 1.0;
    markers.color.a = 1.0;

    markers.scale.x = 0.015;

    for (int i = 0; i < scan.ranges.size(); i++) {
        double theta = scan.angle_min + i * scan.angle_increment;

        geometry_msgs::Point p;
        p.x = scan.ranges.at(i) * std::cos(theta);
        p.y = scan.ranges.at(i) * std::sin(theta);

        markers.points.push_back(p);
    }

    visu_segment_publisher_.publish(markers);
}

void ScanFilterVisualization::visualizeAlignedBeams(std::vector<beam_t> laser_beams)
{
    visualization_msgs::Marker markers;

    markers.header.frame_id = "map";
    markers.header.stamp = ros::Time::now();
    markers.ns = "laser_beams_aligned";
    markers.action = visualization_msgs::Marker::ADD;
    markers.pose.orientation.w = 1.0;

    markers.id = 0;
    
    markers.type = visualization_msgs::Marker::POINTS;

    // points are green
    markers.color.r = 0.0;
    markers.color.g = 1.0;
    markers.color.b = 0.0;
    markers.color.a = 0.7;

    markers.scale.x = 0.07;

    for (int i = 0; i < laser_beams.size(); i++) {
        beam_t beam = laser_beams.at(i);

        geometry_msgs::Point p;
        p.x = beam.endpoint_x;
        p.y = beam.endpoint_y;

        markers.points.push_back(p);
    }

    visu_segment_publisher_.publish(markers);
}

void ScanFilterVisualization::visualizeUnalignedBeams(std::vector<beam_t> laser_beams)
{
    visualization_msgs::Marker markers;

    markers.header.frame_id = "map";
    markers.header.stamp = ros::Time::now();
    markers.ns = "laser_beams_unaligned";
    markers.action = visualization_msgs::Marker::ADD;
    markers.pose.orientation.w = 1.0;

    markers.id = 0;
    
    markers.type = visualization_msgs::Marker::POINTS;

    // points are red
    markers.color.r = 1.0;
    markers.color.g = 0.0;
    markers.color.b = 0.0;
    markers.color.a = 1.0;

    markers.scale.x = 0.04;

    for (int i = 0; i < laser_beams.size(); i++) {
        beam_t beam = laser_beams.at(i);

        geometry_msgs::Point p;
        p.x = beam.endpoint_x;
        p.y = beam.endpoint_y;

        markers.points.push_back(p);
    }

    visu_segment_publisher_.publish(markers);
}

void ScanFilterVisualization::visualizeFilteredScan(std::vector<beam_t> laser_beams)
{
    visualization_msgs::Marker markers;

    markers.header.frame_id = MAP_FRAME_;
    markers.header.stamp = ros::Time::now();
    markers.ns = "filtered_scan";
    markers.action = visualization_msgs::Marker::ADD;
    markers.pose.orientation.w = 1.0;

    markers.id = 0;
    
    markers.type = visualization_msgs::Marker::POINTS;

    // points are blue
    markers.color.r = 0.0;
    markers.color.g = 0.0;
    markers.color.b = 1.0;
    markers.color.a = 0.9;

    markers.scale.x = 0.025;

    for (int i = 0; i < laser_beams.size(); i++) {
        beam_t beam = laser_beams.at(i);

        geometry_msgs::Point p;
        p.x = beam.endpoint_x;
        p.y = beam.endpoint_y;

        markers.points.push_back(p);
    }

    visu_segment_publisher_.publish(markers);
}

void ScanFilterVisualization::visualizeSegements(eyes_msgs::SegmentList& segments)
{
    float rgb[5][3] = {
        { 1.0, 0.0, 0.0 },
        { 0.5, 0.5, 0.0 },
        { 0.0, 1.0, 0.0 },
        { 0.0, 0.5, 0.5 },
        { 0.0, 0.0, 1.0 }
    };

    if (segments.segments.size() > maxSegmentCount)
    {
        maxSegmentCount = segments.segments.size();
    }

    for (int i = 0; i < maxSegmentCount; i++) {
        visualization_msgs::Marker line_strip;

        line_strip.header.frame_id = MAP_FRAME_;
        line_strip.header.stamp = ros::Time::now();
        line_strip.ns = "points_and_lines";
        line_strip.action = visualization_msgs::Marker::ADD;
        line_strip.pose.orientation.w = 1.0;

        line_strip.id = i;
        
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;

        // Line strip is blue
        line_strip.color.r = rgb[i % 5][0];
        line_strip.color.g = rgb[i % 5][1];
        line_strip.color.b = rgb[i % 5][2];
        line_strip.color.a = 1.0;

        line_strip.scale.x = 0.1;

        if (i < segments.segments.size()) {
            for (auto &point: segments.segments[i].points) {
                geometry_msgs::Point p;
                p.x    = point.x;
                p.y    = point.y;

                line_strip.points.push_back(point);
            }
        } else {
            line_strip.color.a = 0.0;
            geometry_msgs::Point p;
            p.x = 0.0;
            p.y = 0.0;

            // at least two points are required
            line_strip.points.push_back(p);
            line_strip.points.push_back(p);
        }

        visu_segment_publisher_.publish(line_strip);
    }
}
