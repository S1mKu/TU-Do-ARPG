#include "control_command_handle_visualization.h"

ControlCommandHandleVisualization::ControlCommandHandleVisualization(ros::NodeHandle& nh):
    node_handle_(nh) {
    loadParameter();
    setupPublisher();
} 

void ControlCommandHandleVisualization::loadParameter() {
    if (!node_handle_.param<std::string>("tf/frames/car_frame", MODEL_BASE_LINK_, "")) 
        ROS_WARN_STREAM("[CCHV] Did not load baselink");   
    if (!node_handle_.param<std::string>("tf/frames/map_frame", MAP_FRAME_, ""))
        ROS_WARN_STREAM("[CCHV] Did not load map frame.");  
}

void ControlCommandHandleVisualization::setupPublisher() {
    this->visu_publisher_ = 
        this->node_handle_.advertise<visualization_msgs::Marker>(
            "/test/visu/pp_goal", 
            1
        );
}

void ControlCommandHandleVisualization::visualize() { }

void ControlCommandHandleVisualization::goalPoint(const geometry_msgs::Point32 point) {

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

  visu_publisher_.publish(marker);

}
