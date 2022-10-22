#pragma one

#include <string>
#include <cstddef>
#include <limits>

#include <ros/ros.h>
#include <ros/console.h>

#include "nav_msgs/OccupancyGrid.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>

#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>

#include "background_substraction.hpp"
#include "segmentation.hpp"
#include "obstacle_detection_lidar_visualization.hpp"
#include "scan_matching.hpp"

// #include "f1tenth_gym_agent/OdomScan.h"
#include "eyes_msgs/PoseWithScan.h"
#include "eyes_msgs/SegmentList.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Polygon.h"

#include "centerline_service/Centerline.h"

#include "grid_map.hpp"

#include "nav_msgs/GetMap.h"

#include "data.hpp"

class ScanFilter {
public:
    ScanFilter(ros::NodeHandle& private_nh, ros::NodeHandle& nh);

    tf::TransformListener tf_listener;

private:
    map_t* map_ = nullptr;

    ros::NodeHandle node_handle_;
    ros::NodeHandle private_node_handle_;
    tf::TransformListener listener_;

    void loadParameter();
    std::string ns;
    std::string LASER_FRAME_;
    std::string MODEL_BASE_LINK_;
    std::string MAP_FRAME_;
    std::string PUBLISH_FINAL_TOPIC_;
    std::string SUBSCRIBE_MAP_TOPIC_;
    std::string SUBSCRIBE_SCAN_TOPIC_;

    double laser_tf_x = std::numeric_limits<double>::infinity();
    double laser_tf_y = std::numeric_limits<double>::infinity();

    bool no_obstacle_flag_ = false;

    nav_msgs::Odometry cur_pos;

    void setupSubscriber();
    ros::Subscriber scan_subscriber_;
    ros::Subscriber map_subscriber_;
    ros::Subscriber pose_estimate_subscriber;
    // ros::Subscriber odom_scan_subscriber;
    ros::Subscriber pose_with_scan_subscriber;

    void setupPublisher();
    ros::Publisher filtered_scan_publisher_;
    ros::Publisher pub_waypoints;
    ros::Publisher pub_costmap;
    ros::Publisher rviz_pub;

    ros::ServiceClient srv_centerline_client;

    nav_msgs::GetMap::Response map_resp;

    std::string ns_;

    geometry_msgs::Polygon centerline_poly;

    laser_geometry::LaserProjection projector_;
    tf::TransformListener tf_listener_;

    OccupancyBackgroundSubtraction background_subtraction_;
    AdaptiveBreakpointDetection segmentation_;
    ICP scan_matching_;

    bool visualize_;
    ScanFilterVisualization scan_filter_visualization_;

    bool calc_map_ = true;

    std::vector<beam_t> preprocessScan(const sensor_msgs::LaserScan& laserscan, nav_msgs::Odometry& ego_pose);

    /**
     * @brief Does a background subtraction based on incoming pointcloud. 
     * The pointcloud has to be converted to the map frame beforehand.
     * @param laserscan input laserscan from lidar of simulator
     */
    void scanCallback(sensor_msgs::LaserScan *laserscan);

    void poseWithScanCallback(const eyes_msgs::PoseWithScan::ConstPtr& msg);

    void poseEstimateCallback(geometry_msgs::PoseWithCovariance *msg);

    /**
     * @brief Saves incoming OccupancyGrid
     * @param map The occupancyGrid Map from TOPIC_MAP
     */
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg);

};