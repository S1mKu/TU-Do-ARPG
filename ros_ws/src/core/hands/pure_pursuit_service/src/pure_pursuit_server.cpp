#include <ros/ros.h>

#include "hands_srv/Pure_Pursuit.h"

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

#include "pure_pursuit_visualization.h"

#include "spline/spline.hpp"

// #######################################
// Params for service
std::string SERVICE_NAME_;
std::string EGO_ID_;
// Car Param
double CAR_LENGTH_;
// Pure Pursuit Parame
double MAX_LOOK_AHEAD_;
double MIN_LOOK_AHEAD_;
double SHIFT_FRONT_POINT_;
double DESIRED_TIME_DELTA_;

std::shared_ptr<PurePursuitVisualization> viz_;

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
double v_max = 20.0;
double a_max = 9.51;
double a_min = (-1) * a_max;
double mu = 1.0489; // friction coefficient
double m = 3.74; // mass of the vehicle
double g = 9.81; // gravity

int goal_point_index = 0;
int cte_index = 0;
double cte = 0.0;

std::string CTE_TOPIC_;
ros::Publisher cte_publisher_;
std_msgs::Float32 cte_message_;




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



void publishMetrics(double cte) {
    ROS_WARN("[PP] %s : cte: %f", EGO_ID_.c_str(), cte);
    cte_message_.data = cte;
    cte_publisher_.publish(cte_message_);
}

double calcuatePurePursuitAngle(
    geometry_msgs::PolygonStamped& centerline,
    geometry_msgs::PoseWithCovarianceStamped& pose_stamped,
    double old_angle,
    double current_target_v
) {

    // Stop the car if the trajectory is empty
    if (centerline.polygon.points.empty()) {
        //TODO does not stop the car. Throw exception to stop the car
        ROS_WARN_STREAM("[PPS] Trajectory is empty");
        return 0.0;
    }

    tf::Quaternion quat;

    tf::quaternionMsgToTF(pose_stamped.pose.pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    
    double orientation = yaw;

    // calculate rear axle coordinates
    // TODO: load distance [cog -> rear] from car config
    geometry_msgs::Point rear_point;
    rear_point.x = pose_stamped.pose.pose.position.x - SHIFT_FRONT_POINT_ * std::cos(orientation);
    rear_point.y = pose_stamped.pose.pose.position.y - SHIFT_FRONT_POINT_ * std::sin(orientation);

    // calculate look ahead point
    geometry_msgs::Point look_ahead_point;
    double relative_speed = current_target_v / (0.5 * v_max);
    double adaptive_look_ahead = DESIRED_TIME_DELTA_ * current_target_v;
    if (adaptive_look_ahead < MIN_LOOK_AHEAD_) {
        adaptive_look_ahead = MIN_LOOK_AHEAD_;
    } else if (adaptive_look_ahead > MAX_LOOK_AHEAD_) {
        adaptive_look_ahead = MAX_LOOK_AHEAD_;
    }
    ROS_WARN("[PPS] Adaptive look_ahead_distance: %f", adaptive_look_ahead);
    look_ahead_point.x =
        adaptive_look_ahead * std::cos(orientation) + rear_point.x;
    look_ahead_point.y =
        adaptive_look_ahead * std::sin(orientation) + rear_point.y;

    const auto look_ahead_iterator = std::min_element(
        centerline.polygon.points.begin(), centerline.polygon.points.end(),
        [&](const geometry_msgs::Point32& trj_point_a,
            const geometry_msgs::Point32& trj_point_b) {
            // Calculate the closest point to the look up point
            double compare_point_a =
                std::hypot(look_ahead_point.x - trj_point_a.x,
                        look_ahead_point.y - trj_point_a.y);
            double compare_point_b =
                std::hypot(look_ahead_point.x - trj_point_b.x,
                        look_ahead_point.y - trj_point_b.y);

            return compare_point_a < compare_point_b;
    });

    geometry_msgs::Point cte_point;
    cte_point.x = pose_stamped.pose.pose.position.x;
    cte_point.y = pose_stamped.pose.pose.position.y;
    const auto cte_iterator = std::min_element(
        centerline.polygon.points.begin(), centerline.polygon.points.end(),
        [&](const geometry_msgs::Point32& trj_point_a,
            const geometry_msgs::Point32& trj_point_b) {
            // Calculate the closest point to the look up point
            double compare_point_a =
                std::hypot(cte_point.x - trj_point_a.x,
                        cte_point.y - trj_point_a.y);
            double compare_point_b =
                std::hypot(cte_point.x - trj_point_b.x,
                        cte_point.y - trj_point_b.y);

            return compare_point_a < compare_point_b;
    });

    // goal point with look ahead distance (as index)
    const int index_goal_point =
        std::distance(centerline.polygon.points.begin(), look_ahead_iterator);

    goal_point_index = index_goal_point;

    const int cte_goal_index =
        std::distance(centerline.polygon.points.begin(), cte_iterator);

    cte_index = cte_goal_index;

    geometry_msgs::Point32 cte_goal_point = centerline.polygon.points.at(cte_goal_index);
    geometry_msgs::Point32 cte_goal_point_plus_one = centerline.polygon.points.at(cte_goal_index + 1);
    // double distance = sqrt(pow((cte_goal_point.x-cte_point.x), 2) + pow((cte_goal_point.y - cte_point.y), 2));
    // bool isLeft = ((cte_goal_point_plus_one.x - cte_goal_point.x)*(cte_point.y - cte_goal_point.y) - (cte_goal_point_plus_one.y - cte_goal_point.y) * (cte_point.x - cte_goal_point.x)) > 0;

    double normal_x = -1 * (cte_goal_point_plus_one.y - cte_goal_point.y);
    double normal_y = cte_goal_point_plus_one.x - cte_goal_point.x;
    double pq_x =  cte_goal_point.x - cte_point.x;
    double pq_y = cte_goal_point.y - cte_point.y;

    double dot = normal_x * pq_x + normal_y * pq_y;

    double e_dot = dot / (std::sqrt(std::pow(normal_x, 2)+std::pow(normal_y, 2)));

    // cte = distance;
    // if (isLeft) {
    //     cte = -distance;
    // }

    publishMetrics(e_dot);

    const double beta = old_angle * 0.5;

    // Calculate angle between look up point and clostest point on the path
    double alpha =
        std::atan2(  centerline.polygon.points[index_goal_point].y - rear_point.y,
                centerline.polygon.points[index_goal_point].x - rear_point.x) -
            (orientation + beta);

    viz_->visualize(centerline, centerline.polygon.points[index_goal_point]);

    return (2 * CAR_LENGTH_ * std::sin(alpha));
}


// The below process is performed when there is a service request
// The service request is declared as 'req', and the service response is declared as 'res'
bool calculate_control_command(
    hands_srv::Pure_Pursuit::Request &req,
    hands_srv::Pure_Pursuit::Response &res) {

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "raceline";
    marker.action = visualization_msgs::Marker::ADD;      
    marker.type = visualization_msgs::Marker::ARROW;

    // arrows are red
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    marker.scale.x = 0.2;    // arrow length
    marker.scale.y = 0.04;    // arrow width
    marker.scale.z = 0.1;   // arrow height

    int N = v.size();
    for (int i = 0; i < N; i++)
    {
        marker.points.clear();
        geometry_msgs::Point32 p = centerline.polygon.points.at(i);
        double theta = psi.at(i); // std::atan2(Y_1st_deriv.at(i), X_1st_deriv.at(i));

        tf2::Quaternion quat_theta;
        quat_theta.setRPY(0, 0, theta);
        marker.pose.orientation = tf2::toMsg(quat_theta);
        marker.pose.position.x  = p.x;
        marker.pose.position.y  = p.y;
        marker.pose.position.z  = 0;

        marker.id = i;
        rviz_pub.publish(marker);
    }

    double min_v = 20.0;
    double max_v = 0;

    for (int i = 0; i < v.size(); i++)
    {
        if (v.at(i) > max_v)
        {
            max_v = v.at(i);
        }
        else if (v.at(i) < min_v)
        {
            min_v = v.at(i);
        }
    }

    marker.ns = "raceline_vel";
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.id = 0;

    // arrows are red
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    marker.scale.x = 0.2;    // arrow length
    marker.scale.y = 0.2;    // arrow width
    marker.scale.z = 0.1;   // arrow height

    for (int i = 0; i < N; i++)
    {
        marker.points.clear();
        marker.scale.z = 0.1 + ((v.at(i) - min_v) / (max_v - min_v));
        geometry_msgs::Point p;
        p.x = centerline.polygon.points.at(i).x;
        p.y = centerline.polygon.points.at(i).y;
        p.z = 0.0;

        marker.points.push_back(p);
        marker.id = i;
        rviz_pub.publish(marker);
    }

    try {
        double steer = 0.0;

        if (req.trajectory.polygon.points.size() == 0)
        {
            steer = calcuatePurePursuitAngle(
                // req.trajectory,
                centerline,
                req.pose,
                req.old_angle.data,
                req.target_velocity.data
            );
        }
        else
        {
            steer = calcuatePurePursuitAngle(
                req.trajectory,
                req.pose,
                req.old_angle.data,
                req.target_velocity.data
            );
        }

        res.steering_angle.data = steer;
        res.speed.data = v.at(goal_point_index) * 0.60;

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
        ROS_WARN_STREAM("[PPV] Did not load car_length");
        if (!nh.param<double>("params/desired_time_delta", DESIRED_TIME_DELTA_, 0.5)) 
        ROS_WARN_STREAM("[PPV] Did not load desired_time_delta");
    if (!nh.param<double>("params/max_look_ahead", MAX_LOOK_AHEAD_, 8.0)) 
        ROS_WARN_STREAM("[PPV] Did not load max_look_ahead");
        if (!nh.param<double>("params/min_look_ahead", MIN_LOOK_AHEAD_, 1.0)) 
        ROS_WARN_STREAM("[PPV] Did not load min_look_ahead");
    if (!nh.param<double>("params/shift_rear_point", SHIFT_FRONT_POINT_, 0.0)) 
        ROS_WARN_STREAM("[PPV] Did not load shift_rear_point");
    if (!nh.param<std::string>("services/advertise", SERVICE_NAME_, "")) 
        ROS_WARN_STREAM("[PPV] Did not load advertise");
    if (!nh.param<std::string>("topics/publish/cte_topic", CTE_TOPIC_, "")) 
        ROS_WARN_STREAM("[PID] Did not load cte_topic");
}

void setupPublisher(ros::NodeHandle& nh) {
    cte_publisher_ = 
        nh.advertise<std_msgs::Float32>(
            CTE_TOPIC_, 1, true
        );
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pure_pursuit_server");
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
        ROS_WARN_STREAM("[PP] Success loading raceline");

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
        ROS_WARN_STREAM("[PP] Cannot call raceline from service");
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

    viz_ = std::shared_ptr<PurePursuitVisualization>(new PurePursuitVisualization(private_nh));

    ros::ServiceServer pure_pursuit_server 
    = nh.advertiseService(SERVICE_NAME_, calculate_control_command);

    ROS_WARN("[PPS] Ready pure_puruit src server!");

    // Wait for the service request
    ros::spin(); 

    return 0;
}
