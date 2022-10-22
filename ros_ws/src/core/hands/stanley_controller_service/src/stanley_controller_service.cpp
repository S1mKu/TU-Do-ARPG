#include <ros/ros.h>

#include "hands_srv/Stanley_Controller.h"

#include "geometry_msgs/Polygon.h"
#include "centerline_service/Centerline.h"
#include "raceline_service/Raceline.h"
#include "geometry_msgs/PolygonStamped.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include <visualization_msgs/MarkerArray.h>

#include <geometry_msgs/Point.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "tf/transform_datatypes.h"

#include "stanley_controller_visualization.h"

#include "spline/spline.hpp"

// #######################################
// Params for service
std::string SERVICE_NAME_;
std::string EGO_ID_;
// Car Param
double CAR_LENGTH_;
// Pure Pursuit Parame
double SHIFT_FRONT_POINT_;

int cte_index = 0;
double cte = 0.0;

std::string CTE_TOPIC_;
ros::Publisher cte_publisher_;
std_msgs::Float32 cte_message_;

std::string YAW_DELTA_TOPIC_;
ros::Publisher yaw_delta_publisher_;
std_msgs::Float32 yaw_delta_message_;

std::shared_ptr<StanleyControllerVisualization> viz_;

ros::Publisher rviz_pub;
// ros::ServiceClient srv_centerline_client;
ros::ServiceClient srv_raceline_client;
geometry_msgs::PolygonStamped centerline;

std::vector<double> curvatures;
std::vector<double> velocities;
std::vector<double> dist_to_successor;

std::vector<double> X_1st_deriv;
std::vector<double> Y_1st_deriv;

std::vector<double> X_2nd_deriv;
std::vector<double> Y_2nd_deriv;

std::vector<double> angle_diff;

// f110_env.py:123
double v_max = 15.0;
double a_max = 9.51;
double a_min = (-1) * a_max;
double mu = 1.0489; // friction coefficient
double m = 3.74; // mass of the vehicle
double g = 9.81; // gravity
double MAX_STEERING_DELTA_ = 10.0;

double e_k_;
double soft_k_;
double yaw_k_;

int goal_point_index = 0;




std::vector<double> s_m;
std::vector<double> x_m;
std::vector<double> y_m;
std::vector<double> psi;
std::vector<double> kappa;
std::vector<double> v;
std::vector<double> a;


// ########################################

void computeMaxVelocities() {
    int N = x_m.size();

    v = std::vector<double>(N, v_max);
    
    for (int n = 2 * N - 2; n >= 0; n--)
    {
        int i = (n + 1) % N; // index of next point in centerline
        int j = n % N;

        double v_i = v.at(i);
        double k_j = kappa.at(j);
        double d = s_m.at(i) - s_m.at(j);

        if (d > 1.0)
        {
            d = s_m.at(1) - s_m.at(0);
        }

        double v_k_max = std::sqrt(mu * g / std::abs(k_j));
        double v_a_max = std::sqrt((v_i * v_i) - 2 * a_min * d);

        double v_j = std::min(std::min(v_k_max, v_a_max), v_max);
        
        v.at(j) = v_j;
    }
}

void publishMetrics(double cte, double yaw_delta) {
    ROS_WARN("[PP] %s : cte: %f, yaw_delta: %f", EGO_ID_.c_str(), cte, yaw_delta);
    cte_message_.data = cte;
    cte_publisher_.publish(cte_message_);

    yaw_delta_message_.data = yaw_delta;
    yaw_delta_publisher_.publish(yaw_delta_message_);
}

double calculateStanleyAngle(
    geometry_msgs::PolygonStamped& centerline,
    geometry_msgs::PoseWithCovarianceStamped& pose_stamped,
    double old_angle,
    double current_target_v
) {

    // Stop the car if the trajectory is empty
    if (centerline.polygon.points.empty()) {
        //TODO does not stop the car. Throw exception to stop the car
        ROS_WARN_STREAM("[SCS] Trajectory is empty");
        return 0.0;
    }

    tf::Quaternion quat;

    tf::quaternionMsgToTF(pose_stamped.pose.pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    
    double orientation = yaw;

    // calculate rear axle coordinates
    // TODO: load distance [cog -> rear] from car config
    geometry_msgs::Point front_point;
    front_point.x = pose_stamped.pose.pose.position.x - SHIFT_FRONT_POINT_ * std::cos(orientation);
    front_point.y = pose_stamped.pose.pose.position.y - SHIFT_FRONT_POINT_ * std::sin(orientation);


    const auto look_ahead_iterator = std::min_element(
        centerline.polygon.points.begin(), centerline.polygon.points.end(),
        [&](const geometry_msgs::Point32& trj_point_a,
            const geometry_msgs::Point32& trj_point_b) {
            // Calculate the closest point to the look up point
            double compare_point_a =
                std::hypot(front_point.x - trj_point_a.x,
                        front_point.y - trj_point_a.y);
            double compare_point_b =
                std::hypot(front_point.x - trj_point_b.x,
                        front_point.y - trj_point_b.y);

            return compare_point_a < compare_point_b;
    });

    // goal point with look ahead distance (as index)
    const int index_goal_point =
        std::distance(centerline.polygon.points.begin(), look_ahead_iterator);

    goal_point_index = index_goal_point;

    viz_->visualize(centerline, centerline.polygon.points[index_goal_point]);

    geometry_msgs::Point32 goal_point = centerline.polygon.points.at(index_goal_point);
    geometry_msgs::Point32 goal_point_plus_one = centerline.polygon.points.at(index_goal_point + 1);
    double normal_x = -1 * (goal_point_plus_one.y - goal_point.y);
    double normal_y = goal_point_plus_one.x - goal_point.x;
    double pq_x =  goal_point.x - front_point.x;
    double pq_y = goal_point.y - front_point.y;

    double dot = normal_x * pq_x + normal_y * pq_y;

    double e_dot = dot / (std::sqrt(std::pow(normal_x, 2)+std::pow(normal_y, 2)));

    ROS_WARN("[SCS] e_dot: %f", e_dot);

    double traj_orientation = std::atan2(goal_point_plus_one.y - goal_point.y, goal_point_plus_one.x - goal_point.x);

    double orientation_diff = traj_orientation - orientation;

    if (orientation_diff > M_PI) {
        orientation_diff += - 2*M_PI;
    }
    else if (orientation_diff < -M_PI) {
        orientation_diff += 2*M_PI;
    }

    // ROS_WARN("[SCS] Orientation_Diff: %f, traj_orientation: %f, orientation: %f", orientation_diff, traj_orientation, orientation);

    // double distance = sqrt(pow(goal_point.x-front_point.x, 2) + pow(goal_point.y - front_point.y, 2));

    // bool isLeft = ((goal_point_plus_one.x - goal_point.x)*(front_point.y - goal_point.y) - (goal_point_plus_one.y - goal_point.y) * (front_point.x - goal_point.x)) > 0;

    // double e = distance;

    // if (isLeft) {
    //     e = -distance;
    // }

    double desired_angle = orientation_diff + std::atan2( (e_k_ * e_dot), (soft_k_ + current_target_v));
    // double steer_diff = desired_angle - old_angle;
    // double max_steering_delta = MAX_STEERING_DELTA_ * (1 / 10 * std::pow(current_target_v, 2));

    // if ( steer_diff > max_steering_delta) {
    //     desired_angle = old_angle + max_steering_delta;
    // } else if ( steer_diff < -max_steering_delta) {
    //     desired_angle = old_angle - max_steering_delta;
    // }

    publishMetrics(e_dot, orientation_diff);

    return desired_angle;
}


// The below process is performed when there is a service request
// The service request is declared as 'req', and the service response is declared as 'res'
bool calculate_control_command(
    hands_srv::Stanley_Controller::Request &req,
    hands_srv::Stanley_Controller::Response &res) {

    // visualization_msgs::Marker marker;
    // marker.header.frame_id = "map";
    // marker.header.stamp = ros::Time::now();
    // marker.ns = "raceline";
    // marker.action = visualization_msgs::Marker::ADD;      
    // marker.type = visualization_msgs::Marker::ARROW;

    // // arrows are red
    // marker.color.r = 1.0;
    // marker.color.g = 0.0;
    // marker.color.b = 0.0;
    // marker.color.a = 1.0;

    // marker.scale.x = 0.2;    // arrow length
    // marker.scale.y = 0.04;    // arrow width
    // marker.scale.z = 0.1;   // arrow height

    // int N = v.size();
    // for (int i = 0; i < N; i++)
    // {
    //     marker.points.clear();
    //     geometry_msgs::Point32 p = centerline.polygon.points.at(i);
    //     double theta = psi.at(i); // std::atan2(Y_1st_deriv.at(i), X_1st_deriv.at(i));

    //     tf2::Quaternion quat_theta;
    //     quat_theta.setRPY(0, 0, theta);
    //     marker.pose.orientation = tf2::toMsg(quat_theta);
    //     marker.pose.position.x  = p.x;
    //     marker.pose.position.y  = p.y;
    //     marker.pose.position.z  = 0;

    //     marker.id = i;
    //     rviz_pub.publish(marker);
    // }

    // double min_v = 20.0;
    // double max_v = 0;

    // for (int i = 0; i < v.size(); i++)
    // {
    //     if (v.at(i) > max_v)
    //     {
    //         max_v = v.at(i);
    //     }
    //     else if (v.at(i) < min_v)
    //     {
    //         min_v = v.at(i);
    //     }
    // }

    // marker.ns = "raceline_vel";
    // marker.type = visualization_msgs::Marker::CYLINDER;
    // marker.id = 0;

    // // arrows are red
    // marker.color.r = 1.0;
    // marker.color.g = 0.0;
    // marker.color.b = 0.0;
    // marker.color.a = 1.0;

    // marker.scale.x = 0.2;    // arrow length
    // marker.scale.y = 0.2;    // arrow width
    // marker.scale.z = 0.1;   // arrow height

    // for (int i = 0; i < N; i++)
    // {
    //     marker.points.clear();
    //     marker.scale.z = 0.1 + ((v.at(i) - min_v) / (max_v - min_v));
    //     geometry_msgs::Point p;
    //     p.x = centerline.polygon.points.at(i).x;
    //     p.y = centerline.polygon.points.at(i).y;
    //     p.z = 0.0;

    //     marker.points.push_back(p);
    //     marker.id = i;
    //     rviz_pub.publish(marker);
    // }

    try {
        double steer = 0.0;

        if (req.trajectory.polygon.points.size() == 0)
        {
            steer = calculateStanleyAngle(
                // req.trajectory,
                centerline,
                req.pose,
                req.old_angle.data,
                req.target_velocity.data
            );
        }
        else
        {
            steer = calculateStanleyAngle(
                req.trajectory,
                req.pose,
                req.old_angle.data,
                req.target_velocity.data
            );
        }

        res.steering_angle.data = steer;
        res.speed.data = v.at(goal_point_index) * 0.60;
        // res.speed.data = v.at(goal_point_index);

        // ROS_WARN_STREAM("v: " << velocities.at(goal_point_index) << "  k: " << 1.0 / curvatures.at(goal_point_index) << "  theta: " << std::atan2(Y_1st_deriv.at(goal_point_index), X_1st_deriv.at(goal_point_index)) / M_PI * 180.0);

        return true;
    }
    catch(const std::exception& e) {
        std::cerr << e.what() << '\n';
        return false;
    }
}

void loadParameter(ros::NodeHandle& nh) {
    EGO_ID_ = ros::this_node::getNamespace();

    ROS_WARN_STREAM("[PP]: namespace: " << EGO_ID_);

    if (!nh.param<double>("params/car_length", CAR_LENGTH_, 0.0)) 
        ROS_WARN_STREAM("[SCS] Did not load car_length");
    if (!nh.param<double>("params/shift_front_point", SHIFT_FRONT_POINT_, 0.0)) 
        ROS_WARN_STREAM("[SCS] Did not load shift_front_point");
    if (!nh.param<double>("params/e_k", e_k_, 0.0)) 
        ROS_WARN_STREAM("[SCS] Did not load e_k");
    if (!nh.param<double>("params/soft_k", soft_k_, 0.0)) 
        ROS_WARN_STREAM("[SCS] Did not load soft_k");
    if (!nh.param<double>("params/yaw_k", yaw_k_, 0.0)) 
        ROS_WARN_STREAM("[SCS] Did not load yaw_k");
    if (!nh.param<std::string>("services/advertise", SERVICE_NAME_, "")) 
        ROS_WARN_STREAM("[SCS] Did not load advertise");
    if (!nh.param<double>("params/max_steering_delta", MAX_STEERING_DELTA_, 10.0)) 
        ROS_WARN_STREAM("[SCS] Did not load max_steering_delta");
    if (!nh.param<std::string>("topics/publish/cte_topic", CTE_TOPIC_, "")) 
        ROS_WARN_STREAM("[PID] Did not load cte_topic");
    if (!nh.param<std::string>("topics/publish/yaw_delta_topic", YAW_DELTA_TOPIC_, "")) 
        ROS_WARN_STREAM("[PID] Did not load yaw_delta_topic");
}

void setupPublisher(ros::NodeHandle& nh) {
    cte_publisher_ = 
        nh.advertise<std_msgs::Float32>(
            CTE_TOPIC_, 1, true
        );
    yaw_delta_publisher_ = 
        nh.advertise<std_msgs::Float32>(
            YAW_DELTA_TOPIC_, 1, true
        );
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "stanley_controller_server");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    loadParameter(private_nh);
    setupPublisher(private_nh);

    rviz_pub = private_nh.advertise<visualization_msgs::Marker>("velocity", 1000);

    srv_raceline_client = nh.serviceClient<raceline_service::Raceline>("/get_raceline");
    srv_raceline_client.waitForExistence();

    raceline_service::Raceline raceline_srv_resp;

    if (srv_raceline_client.call(raceline_srv_resp))
    {
        ROS_WARN_STREAM("[SCS] Success loading raceline");

        s_m = std::vector<double>(raceline_srv_resp.response.s_m.begin(), raceline_srv_resp.response.s_m.end());
        x_m = std::vector<double>(raceline_srv_resp.response.x_m.begin(), raceline_srv_resp.response.x_m.end());
        y_m = std::vector<double>(raceline_srv_resp.response.y_m.begin(), raceline_srv_resp.response.y_m.end());
        psi = std::vector<double>(raceline_srv_resp.response.psi.begin(), raceline_srv_resp.response.psi.end());
        kappa = std::vector<double>(raceline_srv_resp.response.kappa.begin(), raceline_srv_resp.response.kappa.end());
        v = std::vector<double>(raceline_srv_resp.response.v.begin(), raceline_srv_resp.response.v.end());
        a = std::vector<double>(raceline_srv_resp.response.a.begin(), raceline_srv_resp.response.a.end());

        for (int i = 0; i < x_m.size(); i++)
        {
            geometry_msgs::Point32 p;
            p.x = x_m.at(i);
            p.y = y_m.at(i);

            centerline.polygon.points.push_back(p);
        }
    }
    else
    {
        ROS_WARN_STREAM("[SCS] Cannot call raceline from service");
        exit(1);
    }

    // centerline_service::Centerline centerline_srv_resp;

    // if (srv_centerline_client.call(centerline_srv_resp))
    // {
    //     ROS_WARN_STREAM("[PP] Success loading Centerline");
    //     centerline = centerline_srv_resp.response.centerline;
    // }
    // else
    // {
    //     ROS_WARN_STREAM("[PP] Cannot call centerline from service");
    //     exit(1);
    // }

    // computeCurvatures();
    // computeDistToSuccessor();
    // computeAngleDiff();
    computeMaxVelocities();

    viz_ = std::shared_ptr<StanleyControllerVisualization>(new StanleyControllerVisualization(private_nh));

    ros::ServiceServer stanley_controller_server 
    = nh.advertiseService(SERVICE_NAME_, calculate_control_command);

    ROS_WARN("[SCS] Ready stanley controller src server!");

    // Wait for the service request
    ros::spin(); 

    return 0;
}
