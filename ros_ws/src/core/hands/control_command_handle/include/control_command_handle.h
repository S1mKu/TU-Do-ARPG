#pragma one

#include <string>
#include <vector>
#include <algorithm>

#include <ros/ros.h>
#include <ros/console.h>

#include "control_command_handle_visualization.h"

#include "f1tenth_gym_agent/drive_param.h"

#include "geometry_msgs/PolygonStamped.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"

#include "eyes_msgs/SegmentList.h"

#include "centerline_service/Centerline.h"
#include "hands_srv/Gap_Follower.h"
#include "hands_srv/Pure_Pursuit.h"
#include "hands_srv/Opp_Trajectory.h"
#include "hands_srv/PID_Distance.h"
#include "hands_srv/Stanley_Controller.h"
#include "hands_msgs/OpponentTrajectory.h"

#include <dynamic_reconfigure/server.h>
#include <control_command_handle/control_command_handle_Config.h>
using dc_server = dynamic_reconfigure::Server<control_command_handle::control_command_handle_Config>;

class ControlCommandHandle {

public:
    ControlCommandHandle(ros::NodeHandle &public_nh, ros::NodeHandle &private_nh, dc_server &m_dyn_cfg_server);

private:

    ros::NodeHandle public_nh_;
    ros::NodeHandle private_nh_;

    void loadParameter(dc_server &m_dyn_cfg_server);
    std::string PUBLISH_FINAL_TOPIC_;
    std::string SUBSCRIBE_POSE_TOPIC_;
    std::string SUBSCRIBE_ODOM_TOPIC_;
    std::string SUBSCRIBE_OBSTACLE_TOPIC_;
    std::string SUBSCRIBE_SCAN_TOPIC_;
    std::string SUBSCRIBE_OPP_TRAJECTORY_TOPIC_;
    std::string CENTERLINE_SERVICE_;
    std::string GAP_FOLLOWER_SERVICE_;
    std::string PURE_PURSUIT_SERVICE_;
    std::string STANLEY_CONTROLLER_SERVICE_;
    std::string OPP_TRAJ_SERVICE_;
    std::string PID_DISTANCE_SERVICE_;
    bool USE_STANLEY_;
    int RATE_;
   


    void setupSubscriber();
    ros::Subscriber pose_subscriber_;
    ros::Subscriber odom_subscriber_;
    ros::Subscriber obstacle_subscriber_;
    ros::Subscriber scan_subscriber_;
    ros::Subscriber opp_trajectory_subscriber_;

    void setupPublisher();
    ros::Publisher command_publisher_;

    void setupServiceClients();
    ros::ServiceClient srv_centerline_client_;
    ros::ServiceClient srv_gap_follower_client_;
    ros::ServiceClient srv_pure_pursuit_client_;
    // ros::ServiceClient srv_opp_traj_client_;
    ros::ServiceClient srv_pid_distance_client_;
    ros::ServiceClient srv_stanley_controller_client_;

    hands_srv::Gap_Follower gap_follower_srv_;
    hands_srv::Pure_Pursuit pure_pursuit_srv_;
    hands_srv::PID_Distance pid_distance_srv_;
    hands_srv::Stanley_Controller stanley_controller_srv_;

    nav_msgs::Odometry odom_;
    geometry_msgs::PoseWithCovarianceStamped pose_stamped_;

    geometry_msgs::PolygonStamped centerline_;

    hands_msgs::OpponentTrajectory opp_trajectory_;

    f1tenth_gym_agent::drive_param final_drive_param_;

    bool visualize_;
    ControlCommandHandleVisualization control_command_handle_visualization_;

    eyes_msgs::SegmentList obstacles_;
    double speed_;
    double speed_mod_;
    int crossover_;

    std::string EGO_ID_;

    sensor_msgs::LaserScan scan_;
    void scanCallback(const sensor_msgs::LaserScan& scan);
    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped& pose_stamped);
    void odomCallback(const nav_msgs::Odometry& odom);
    void obstacleCallback(const eyes_msgs::SegmentList& segmentList);
    void oppTrajectoryCallback(const hands_msgs::OpponentTrajectory& trajectory);
    void controlCall();
    void fixedRateControlCall();

    double calculatePurePursuitCommand(const nav_msgs::Odometry& odom);
};