#include "pure_pursuit_visualization.h"
    
PurePursuitVisualization::PurePursuitVisualization(ros::NodeHandle& nh):
    node_handle_(nh) {
    loadParameter();
    setupPublisher();
}

void PurePursuitVisualization::loadParameter() {
    if (!node_handle_.param<std::string>("tf/frames/car_frame", MODEL_BASE_LINK_, "")) 
        ROS_WARN_STREAM("[PPV] Did not load baselink");   
    if (!node_handle_.param<std::string>("tf/frames/map_frame", MAP_FRAME_, ""))
        ROS_WARN_STREAM("[PPV] Did not load map frame.");

    if (!node_handle_.param<std::string>("topics/publish/viz_look_ahead_topic", VIZ_LH_TOPIC_, ""))
        ROS_WARN_STREAM("[PPV] Did not load viz_look_ahead_topic.");
    if (!node_handle_.param<std::string>("topics/publish/viz_followed_path_topic", VIZ_FP_TOPIC_, ""))
        ROS_WARN_STREAM("[PPV] Did not load viz_followed_path_topic.");
}

void PurePursuitVisualization::setupPublisher() {

    this->look_ahead_publisher_ = 
        this->node_handle_.advertise<visualization_msgs::Marker>(
            VIZ_LH_TOPIC_, 
            1
        );

    this->followed_path_publisher_ = 
        this->node_handle_.advertise<nav_msgs::Path>(
            VIZ_FP_TOPIC_, 
            1
        );
}

void PurePursuitVisualization::visualize(geometry_msgs::PolygonStamped polygon, const geometry_msgs::Point32 point) { 
    followed_path(polygon);
    look_ahead(point);
}

void PurePursuitVisualization::followed_path(geometry_msgs::PolygonStamped polygon) {
    nav_msgs::Path path;
    std::vector<geometry_msgs::PoseStamped> poses;

    for (geometry_msgs::Point32 point : polygon.polygon.points) {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "map";
        pose.pose.position.x = point.x;
        pose.pose.position.y = point.y;
        poses.push_back(pose);
    }

    path.header.stamp = ros::Time::now();
    path.header.frame_id = "map";
    path.poses = poses;

    
    followed_path_publisher_.publish(path);
}

void PurePursuitVisualization::look_ahead(const geometry_msgs::Point32 point) {

    visualization_msgs::Marker marker;

    // Basic settings
    marker.type               = visualization_msgs::Marker::SPHERE;
    marker.action             = visualization_msgs::Marker::ADD;
    marker.header.stamp       = ros::Time::now();
    marker.header.frame_id    = "map";
    marker.color.r            = 1.0;
    marker.color.a            = 1.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x            = 0.2;
    marker.scale.y            = 0.2;
    marker.scale.z            = 0.2;
    
    // Marker for goal point
    marker.pose.position.x = point.x;
    marker.pose.position.y = point.y;
    marker.color.b         = 1.0;
    marker.id              = 1;

    look_ahead_publisher_.publish(marker);

}
