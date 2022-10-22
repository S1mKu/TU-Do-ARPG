#include "obstacle_detection_lidar_node.hpp"

#include <chrono>
#include <thread>
#include <limits>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"

#include <boost/circular_buffer.hpp>
#include <iterator>
#include <algorithm>

// #include "absl/memory/memory.h"

#include "nav_msgs/GetMap.h"

ScanFilter::ScanFilter(ros::NodeHandle& private_nh, ros::NodeHandle& nh): 
    private_node_handle_(private_nh),
    node_handle_(nh),  
    background_subtraction_(private_nh),
    segmentation_(private_nh),
    scan_matching_(private_nh),
    scan_filter_visualization_(private_nh) {

    loadParameter();
    setupSubscriber();
    setupPublisher();

    srv_centerline_client = node_handle_.serviceClient<centerline_service::Centerline>("/get_centerline");
    srv_centerline_client.waitForExistence();

    centerline_service::Centerline centerline_srv_resp;

    if (srv_centerline_client.call(centerline_srv_resp))
    {
        ROS_WARN_STREAM("[ObsDet] Success loading Centerline");
        centerline_poly = centerline_srv_resp.response.centerline.polygon;
    }
    else
    {
        ROS_WARN_STREAM("[ObsDet] Cannot call centerline from service");
        exit(1);
    }
}

void ScanFilter::loadParameter()
{
    ns = ros::this_node::getNamespace();
    std::string frame_prefix = "opp_racecar";

    if (ns == "/a1")
    {
        frame_prefix = "ego_racecar";
    }

    /**
     * Be carefull with leading slashes since they could direct you into the wrong namespace!!!
     * Or make sure to use the correct node handle!!!
     * 
     */
    if (!private_node_handle_.param<std::string>("topics/subscribe/map_topic", SUBSCRIBE_MAP_TOPIC_, "")) 
        ROS_DEBUG_STREAM("Did not load map topic.");
    if (!private_node_handle_.param<std::string>("topics/subscribe/scan_topic", SUBSCRIBE_SCAN_TOPIC_, "")) 
        ROS_DEBUG_STREAM("Did not load scan topic.");
    if (!private_node_handle_.param<std::string>("topics/publish/final_filtered_scan_topic", PUBLISH_FINAL_TOPIC_, ""))
        ROS_DEBUG_STREAM("Did not load publish topic.");    
    if (!private_node_handle_.param<bool>("visualize", visualize_, false))
        ROS_DEBUG_STREAM("Did not load visualize boolean.");
    if (!private_node_handle_.param<std::string>("tf/frames/car_frame", MODEL_BASE_LINK_, "/base_link"))
        ROS_DEBUG_STREAM("Did not load base link.");
    if (!private_node_handle_.param<std::string>("tf/frames/laser_frame", LASER_FRAME_, "/laser_model"))
        ROS_DEBUG_STREAM("Did not load laser frame.");
    if (!private_node_handle_.param<std::string>("tf/frames/map_frame", MAP_FRAME_, "/map"))
        ROS_DEBUG_STREAM("Did not load map link.");

    double threshold_;

    if (!private_node_handle_.param<double>("algorithms/bgs/threshold", threshold_, 1.0))
    {
        ROS_DEBUG_STREAM("Did not load bgs theshold ######.");
    }
    
    LASER_FRAME_ = frame_prefix + LASER_FRAME_;
    MODEL_BASE_LINK_ = frame_prefix + MODEL_BASE_LINK_;

    ROS_DEBUG_STREAM("tf laser frame: " << LASER_FRAME_);
    ROS_DEBUG_STREAM("tf base_link frame: " << MODEL_BASE_LINK_);
}

void ScanFilter::setupSubscriber()
{
    map_subscriber_ = node_handle_.subscribe<nav_msgs::OccupancyGrid>(SUBSCRIBE_MAP_TOPIC_, 1, &ScanFilter::mapCallback, this);
    pose_with_scan_subscriber = node_handle_.subscribe<eyes_msgs::PoseWithScan>("pose_with_scan", 10, &ScanFilter::poseWithScanCallback, this);
}

void ScanFilter::setupPublisher()
{
    // global namespace
    filtered_scan_publisher_ = node_handle_.advertise<eyes_msgs::SegmentList>(PUBLISH_FINAL_TOPIC_, 10, true);

    // private namespace
    pub_waypoints = private_node_handle_.advertise<visualization_msgs::Marker>("ego_pose", 10);
    pub_costmap = private_node_handle_.advertise<nav_msgs::OccupancyGrid>("distance_map", 10);

    rviz_pub = private_node_handle_.advertise<visualization_msgs::Marker>("grid_map", 3000);
}

void ScanFilter::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg)
{
    ROS_INFO("Received a %d X %d map @ %.3f m/pix\n",
           map_msg->info.width,
           map_msg->info.height,
           map_msg->info.resolution);

    if (calc_map_)
    {
        std::chrono::_V2::system_clock::time_point t1 = std::chrono::high_resolution_clock::now();

        map_ =  map_alloc();
        ROS_ASSERT(map_);

        map_init(map_, map_msg, &centerline_poly, rviz_pub);

        map_resp.map.header.stamp = ros::Time::now();
        map_resp.map.header.frame_id = map_msg->header.frame_id;
        map_resp.map.info.map_load_time = ros::Time::now();
        map_resp.map.info.width = map_msg->info.width;
        map_resp.map.info.height = map_msg->info.height;
        map_resp.map.info.resolution = map_msg->info.resolution;
        map_resp.map.info.origin.position.x = map_msg->info.origin.position.x;
        map_resp.map.info.origin.position.y = map_msg->info.origin.position.y;
        map_resp.map.info.origin.position.z = 0.0;
        map_resp.map.info.origin.orientation.x = map_msg->info.origin.orientation.x;
        map_resp.map.info.origin.orientation.y = map_msg->info.origin.orientation.y;
        map_resp.map.info.origin.orientation.z = map_msg->info.origin.orientation.z;
        map_resp.map.info.origin.orientation.w = map_msg->info.origin.orientation.w;

        map_resp.map.data.resize(map_resp.map.info.width * map_resp.map.info.height);
        
        for (int i = 0; i < map_->size_x * map_->size_y; i++)
        {
            map_resp.map.data.at(i) = (char) (std::min(map_->cells[i].occ_dist_inner, map_->cells[i].occ_dist_outer) / map_->max_occ_dist * 255.0);
        }

        // pub_costmap.publish(map_resp.map);

        std::chrono::_V2::system_clock::time_point t2 = std::chrono::high_resolution_clock::now();

        ROS_WARN_STREAM("Obstacle Detection - map calculation took " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() << " ms");
    }
}

void ScanFilter::poseWithScanCallback(const eyes_msgs::PoseWithScan::ConstPtr& msg)
{
    // nav_msgs::Odometry odom = msg->odom;
    geometry_msgs::PoseWithCovariance pose = msg->pose;
    sensor_msgs::LaserScan scan = msg->scan;

    scan_filter_visualization_.visualizeScan(scan);

    // poseEstimateCallback(&odom);
    poseEstimateCallback(&pose);
    scanCallback(&scan);
}

void ScanFilter::poseEstimateCallback(geometry_msgs::PoseWithCovariance *msg)
{
    cur_pos.pose.pose.position.x = msg->pose.position.x;
    cur_pos.pose.pose.position.y = msg->pose.position.y;
    cur_pos.pose.pose.position.z = msg->pose.position.z;
    cur_pos.pose.pose.orientation = msg->pose.orientation;

    if (laser_tf_x == std::numeric_limits<double>::infinity())
    {
        try
        {
            tf:: StampedTransform laser_model_tf;
            tf_listener_.lookupTransform(LASER_FRAME_, MODEL_BASE_LINK_, ros::Time(0), laser_model_tf);

            laser_tf_x = laser_model_tf.getOrigin().getX();
            laser_tf_y = laser_model_tf.getOrigin().getY();
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
        }
    }
}

std::vector<beam_t> ScanFilter::preprocessScan(const sensor_msgs::LaserScan& laserscan, nav_msgs::Odometry& ego_pose)
{
    std::vector<beam_t> laser_beams;

    unsigned int n = laserscan.ranges.size();
    double angle_increment = laserscan.angle_increment;
    double angle_min = laserscan.angle_min;
    double angle_max = laserscan.angle_max;
    double range_max = laserscan.range_max;
    double range_min = laserscan.range_min;

    tf::Quaternion quat;
    tf::quaternionMsgToTF(ego_pose.pose.pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    double theta_base = yaw;

    double ego_x = ego_pose.pose.pose.position.x;
    double ego_y = ego_pose.pose.pose.position.y;

    for (unsigned int i = 0; i < n; i++)
    {
        double range = laserscan.ranges[i];

        beam_t beam;
        beam.theta = theta_base + angle_min + i * angle_increment;
        beam.index = i;

        if (range >= range_min && range <= range_max)
        {
            beam.range = range;
            beam.endpoint_x = ego_x + range * std::cos(beam.theta);
            beam.endpoint_y = ego_y + range * std::sin(beam.theta);
            beam.cluster_id = i;

            // beam.endpoint_x = ego_x + laser_tf_x + range * std::cos(beam.theta);
            // beam.endpoint_y = ego_y + laser_tf_y + range * std::sin(beam.theta);

            // beam.endpoint_x = ego_x - laser_tf_x + range * std::cos(beam.theta);
            // beam.endpoint_y = ego_y - laser_tf_y + range * std::sin(beam.theta);

            laser_beams.push_back(beam);
        }
    }

    std::vector<beam_t> filtered_laser_beams;

    // keep track of the last 5 coordinates
    boost::circular_buffer<std::pair<int, int>> coords_backlog(5);
    int cluster_id = 0;

    beam_t beam_last_omitted;

    for (beam_t b : laser_beams)
    {
        beam_t beam = b;

        int i = MAP_GXWX(map_, beam.endpoint_x);
        int j = MAP_GYWY(map_, beam.endpoint_y);

        bool is_unique = true;
        for (unsigned int ind = 0; ind < coords_backlog.size(); ind++)
        {
            std::pair<int, int> coords = coords_backlog.at(ind);
            if (i == coords.first && j == coords.second)
            {
                beam_last_omitted = beam;
                is_unique = false;
                break;
            }
        }

        if (is_unique) // new cell 
        {
            if (beam_last_omitted.index + 1 == beam.index) // index succession ensures the beams are exactly angle_increment degree apart from each other
            {
                int last_i = MAP_GXWX(map_, beam_last_omitted.endpoint_x);
                int last_j = MAP_GYWY(map_, beam_last_omitted.endpoint_y);

                int diff_x = std::abs(i - last_i);
                int diff_y = std::abs(j - last_j);

                if (diff_x > 1 || diff_y > 1) // beams of neighboring cells are not considered for adaptive breakpoint detection 
                {
                    double euclidean_dist = std::sqrt(std::pow(beam.endpoint_x - beam_last_omitted.endpoint_x, 2) + std::pow(beam.endpoint_y - beam_last_omitted.endpoint_y, 2));
                    double threshold = segmentation_.compute_threshold(beam, beam_last_omitted);

                    if (euclidean_dist <= threshold)
                    {
                        beam_last_omitted.cluster_id = cluster_id; // belongs to last cluster since it is not unique
                        filtered_laser_beams.push_back(beam_last_omitted);
                    }
                }
            }
            // else the last beam was marked as unique and thus is exactly angle_increment degree apart from the current beam

            // check neighborhood to decide whether the point belongs into a new cluster
            bool is_neighbor = false;
            for (unsigned int ind = 0; ind < coords_backlog.size(); ind++)
            {
                std::pair<int, int> coords = coords_backlog.at(ind);

                int diff_x = std::abs(i - coords.first);
                int diff_y = std::abs(j - coords.second);

                if (diff_x <= 1 && diff_y <= 1) // neighbor
                {
                    is_neighbor = true;
                    break;
                }
            }

            if (!is_neighbor)
            {
                cluster_id++;
            }

            coords_backlog.push_back(std::pair<int, int>(i, j));
            beam.cluster_id = cluster_id;
            filtered_laser_beams.push_back(beam);
        }
    }

    return filtered_laser_beams;
}

// void ScanFilter::scanCallback(const sensor_msgs::LaserScan::ConstPtr& laserscan)
void ScanFilter::scanCallback(sensor_msgs::LaserScan *laserscan)
{
    if(tf_listener.canTransform(MAP_FRAME_, LASER_FRAME_, laserscan->header.stamp))
    {
        std::chrono::_V2::system_clock::time_point t1 = std::chrono::high_resolution_clock::now();

        tf::StampedTransform ego_pose;

        try
        {
            tf_listener.lookupTransform(MAP_FRAME_, LASER_FRAME_, laserscan->header.stamp, ego_pose);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
        }

        std::vector<beam_t> unaligned_laser_beams = preprocessScan(*laserscan, cur_pos);
        
        // std::vector<beam_t> laser_beams = scan_matching_.compute_svd_icp(map_, unaligned_laser_beams, 10, &centerline_poly);
        std::vector<beam_t> laser_beams = scan_matching_.compute_ls_icp(map_, unaligned_laser_beams, 10, &centerline_poly);
        // std::vector<beam_t> laser_beams = unaligned_laser_beams;

        std::vector<beam_t> filtered_beams = background_subtraction_.backgroundSubstraction(map_, laser_beams);

        eyes_msgs::SegmentList segments;

        if (filtered_beams.size() == 0)
        {
            ROS_WARN_COND(!no_obstacle_flag_, "No obstacles detected in current scan");
            no_obstacle_flag_ = true;
        }
        else
        {
            no_obstacle_flag_ = false;

            tf::Quaternion quat;
            tf::quaternionMsgToTF(cur_pos.pose.pose.orientation, quat);
            double roll, pitch, yaw;
            tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
            double theta = yaw;

            // Compute the segments
            segments = segmentation_.computeSegments(filtered_beams);

            // Publish the final segments
            segments.header.frame_id = MAP_FRAME_;
            segments.header.stamp = laserscan->header.stamp; // keep the stamp of the source (laserscan)
            segments.observer_theta = theta;
            segments.observer_x = cur_pos.pose.pose.position.x;
            segments.observer_y = cur_pos.pose.pose.position.y;
            filtered_scan_publisher_.publish(segments);
        }

        std::chrono::_V2::system_clock::time_point t2 = std::chrono::high_resolution_clock::now();

        ROS_DEBUG_STREAM_DELAYED_THROTTLE_NAMED(10, "obstacle_detection_callback", "exec. time: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count());

        if (visualize_) {
            scan_filter_visualization_.visualizeFilteredScan(filtered_beams);
            scan_filter_visualization_.visualizeSegements(segments);

            scan_filter_visualization_.visualizeUnalignedBeams(unaligned_laser_beams);
            scan_filter_visualization_.visualizeAlignedBeams(laser_beams);

            // move the visualization out of the time tracking zone (s. t3 variable)
            // scan_filter_visualization_.visualizeFilteredScan(filtered_beams);
            // scan_filter_visualization_.visualizeSegements(segments);
        }
    }
}
