#include <ros/ros.h>

#include "hands_srv/Opp_Trajectory.h"

#include "tf/transform_datatypes.h"

#include "geometry_msgs/PolygonStamped.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "eyes_msgs/ObstacleList.h"
#include "hands_msgs/OpponentTrajectory.h"

std::string SERVICE_NAME_;
// Params for trajectory
// TODO Load Params from .yaml or Launchfile
bool USE_GROUND_TRUTH = true; 
int NUMBER_OF_TRAJ_POINTS = 200;
int MIN_OBS_HISTORY = 10;
// TODO use POINT_INTERVAL / rate
int RATE = 10;
double SWITCH_DISTANCE = 5.0;

std::string EGO_ID;
std::string OPP_ID;
std::string SUBSCRIBE_OPP_POSE_TOPIC;
std::string SUBSCRIBE_OBSTACLE_TOPIC;

ros::Subscriber opp_odom_subscriber_;
ros::Subscriber odom_subscriber;
ros::Publisher trajectory_publisher;

geometry_msgs::PolygonStamped opp_trajectory;
geometry_msgs::PoseWithCovarianceStamped ego_pose;
double opp_velocity;

double ego_orientation;
double dist;

geometry_msgs::Point32 new_point;
bool has_new_point = false;

void oppOdomCallback(const geometry_msgs::PoseWithCovarianceStamped& pose_stamped) {
    dist = std::hypot(pose_stamped.pose.pose.position.x - ego_pose.pose.pose.position.x, pose_stamped.pose.pose.position.y - ego_pose.pose.pose.position.y);
    double alpha = std::atan2(pose_stamped.pose.pose.position.y - ego_pose.pose.pose.position.y, pose_stamped.pose.pose.position.x - ego_pose.pose.pose.position.x);

    double angle_diff = std::abs(alpha-ego_orientation);
    // ROS_WARN("[OTS] %s alpha: %f, angle_diff: %f", EGO_ID.c_str(), alpha, angle_diff);

    if (dist <= SWITCH_DISTANCE && (angle_diff < M_PI/2 || angle_diff > 3*M_PI/2)) {
      geometry_msgs::Point32 p = geometry_msgs::Point32();
      p.x = pose_stamped.pose.pose.position.x;
      p.y = pose_stamped.pose.pose.position.y;
      p.z=  pose_stamped.pose.pose.position.z;
      new_point = p;
      has_new_point = true;
    }
    else {
      opp_trajectory.polygon.points.clear();
    }
}

void obstacleCallback(const eyes_msgs::ObstacleList obstacles) {
    eyes_msgs::Obstacle opp_obstacle;

    for (eyes_msgs::Obstacle obs : obstacles.obstacles) {
      if (obs.x.size() > opp_obstacle.x.size()) {
        opp_obstacle = obs;
      }
    }
    if (opp_obstacle.x.size() > MIN_OBS_HISTORY) {
      double opp_x = opp_obstacle.x[0];
      double opp_y = opp_obstacle.y[0];
      dist = std::hypot(opp_x - ego_pose.pose.pose.position.x, opp_y - ego_pose.pose.pose.position.y);
      double alpha = std::atan2(opp_y - ego_pose.pose.pose.position.y, opp_x - ego_pose.pose.pose.position.x);

      double angle_diff = std::abs(alpha-ego_orientation);

      if (dist <= SWITCH_DISTANCE && (angle_diff < M_PI/2 || angle_diff > 3*M_PI/2)) {
        geometry_msgs::Point32 p = geometry_msgs::Point32();
        p.x = opp_x;
        p.y = opp_y;
        p.z=  0;
        new_point = p;
        opp_velocity = opp_obstacle.velocity;
        has_new_point = true;
      }
      else {
        opp_trajectory.polygon.points.clear();
      }
    }
}

void poseCallback(const geometry_msgs::PoseWithCovarianceStamped& pose_stamped) {
  tf::Quaternion quat;

  tf::quaternionMsgToTF(pose_stamped.pose.pose.orientation, quat);
  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  
  ego_orientation = yaw;

  ego_pose = pose_stamped;
}

void publishTrajectory() {
  hands_msgs::OpponentTrajectory msg;
  msg.trajectory = opp_trajectory;
  msg.distance.data = dist;
  msg.velocity.data = opp_velocity;

  trajectory_publisher.publish(msg);
}

bool getTrajectory(
    hands_srv::Opp_Trajectory::Request &req,
    hands_srv::Opp_Trajectory::Response &res) {

  res.trajectory = opp_trajectory;
  res.distance.data = dist;
  res.velocity.data = opp_velocity;
  return true;
}

void loadParameter(ros::NodeHandle& nh) {

    if (!nh.param<int>("params/n_points", NUMBER_OF_TRAJ_POINTS, 200)) 
        ROS_WARN_STREAM("[PID] Did not load n_points");
    if (!nh.param<int>("params/rate", RATE, 10)) 
        ROS_WARN_STREAM("[PID] Did not load rate");
    if (!nh.param<int>("params/min_obs_history", MIN_OBS_HISTORY, 10)) 
        ROS_WARN_STREAM("[PID] Did not load min_obs_history");
    if (!nh.param<double>("params/switch_distance", SWITCH_DISTANCE, 5.0)) 
        ROS_WARN_STREAM("[PID] Did not load switch_distance");
    if (!nh.param<bool>("params/ground_truth", USE_GROUND_TRUTH, true)) 
        ROS_WARN_STREAM("[PID] Did not load ground_truth");
    if (!nh.param<std::string>("services/advertise", SERVICE_NAME_, "")) 
        ROS_WARN_STREAM("[PID] Did not load advertise");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "opp_traj_server");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  loadParameter(private_nh);
  ros::Rate rate(RATE);

  EGO_ID = ros::this_node::getNamespace();
  if (EGO_ID == "/a1") {
    OPP_ID = "/a2";
  }
  if (EGO_ID == "/a2") {
    OPP_ID = "/a1";
  }

  SUBSCRIBE_OPP_POSE_TOPIC = OPP_ID + "/pose";
  SUBSCRIBE_OBSTACLE_TOPIC = EGO_ID + "/obstacle_tracking/obstacles";
  std::string PUBLISH_TOPIC = EGO_ID + "/opponent_trajectory";

  if(USE_GROUND_TRUTH) {
    opp_odom_subscriber_ = nh.subscribe(SUBSCRIBE_OPP_POSE_TOPIC, 1, oppOdomCallback);
  }
  else {
    opp_odom_subscriber_ = nh.subscribe(SUBSCRIBE_OBSTACLE_TOPIC, 1, obstacleCallback);
  }
  
  odom_subscriber = nh.subscribe("pose", 1, poseCallback);
 
  trajectory_publisher = nh.advertise<hands_msgs::OpponentTrajectory>(PUBLISH_TOPIC, 1, true);

  ros::ServiceServer opp_traj_server 
    = nh.advertiseService(SERVICE_NAME_, getTrajectory);

  ROS_WARN("Ready opp_traj_server!");

  // Wait for the service request
  while(ros::ok()) {
    rate.sleep();
    if (has_new_point) {
      opp_trajectory.polygon.points.insert(opp_trajectory.polygon.points.begin(), new_point);

      if (NUMBER_OF_TRAJ_POINTS < opp_trajectory.polygon.points.size() ) {
        opp_trajectory.polygon.points.erase(opp_trajectory.polygon.points.end());
      }

      has_new_point = false;
    }
    publishTrajectory();
    ros::spinOnce();
  }

  return 0;
}
