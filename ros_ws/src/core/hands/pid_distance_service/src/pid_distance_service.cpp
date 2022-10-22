#include <ros/ros.h>

#include "hands_srv/PID_Distance.h"

// #######################################
// Params for service
std::string SERVICE_NAME_;
// PID Params
double K_P_ = 4;
double K_I_ = 0.005;
double K_D_ = 0.5;
double MAX_VELOCITY_ = 4;
double MIN_VELOCITY_ = 0;
double DESIRED_TIME_DELTA_ = 0.5;

// Variables for service
double integral_ = 0;
double pre_error_ = 0;

//Publisher for debug/evaluation
std::string EGO_ID_;

std::string ERROR_TOPIC_;
std::string OUTPUT_TOPIC_;
std::string DISTANCE_TOPIC_;
std::string SET_POINT_TOPIC_;

ros::Publisher error_publisher_;
ros::Publisher output_publisher_;
ros::Publisher distance_publisher_;
ros::Publisher set_point_publisher_;

std_msgs::Float32 error_message_;
std_msgs::Float32 output_message_;
std_msgs::Float32 distance_message_;
std_msgs::Float32 set_point_message_;

// ########################################


void publishMetrics(double error, double output, double distance, double desired_distance){

    ROS_WARN("[PID] %s : distance: %f, desired_distance: %f, error: %f, output: %f", EGO_ID_.c_str() ,distance, desired_distance, error, output);
    error_message_.data = error;
    error_publisher_.publish(error_message_);
    output_message_.data = output;
    output_publisher_.publish(output_message_);
    distance_message_.data = distance;
    distance_publisher_.publish(distance_message_);
    set_point_message_.data = desired_distance;
    set_point_publisher_.publish(set_point_message_);

}

// Calculates desired speed based on distance to the opponent
// implementation from: https://gist.github.com/bradley219/5373998
double calcuate_pid(
    double distance,
    double velocity
) {
    // Calculate desired distance
    double desired_distance = DESIRED_TIME_DELTA_ * velocity;
    
    // Calculate error
    double error = distance - desired_distance;

    // Proportional term
    double Pout = K_P_ * error;

    // Integral term
    integral_ += error;
    double Iout = K_I_ * integral_;

    // Derivative term
    double derivative = (error - pre_error_);
    double Dout = K_D_ * derivative;

    // Calculate total output
    double output = Pout + Iout + Dout;

    // Restrict to max/min
    if( output > MAX_VELOCITY_ )
        output = MAX_VELOCITY_;
    else if( output < MIN_VELOCITY_ )
        output = MIN_VELOCITY_;

    // Save error to previous error
    pre_error_ = error;

    publishMetrics(error, output, distance, desired_distance);

    return output;
  }

// The below process is performed when there is a service request
// The service request is declared as 'req', and the service response is declared as 'res'
bool calculate_velocity(
    hands_srv::PID_Distance::Request &req,
    hands_srv::PID_Distance::Response &res) {

    try {

        double velocity = calcuate_pid(
            req.distance.data,
            req.velocity.data
        );

        res.velocity.data = velocity;

        return true;
    }
    catch(const std::exception& e) {
        std::cerr << e.what() << '\n';
        return false;
    }
}


void loadParameter(ros::NodeHandle& nh) {

    EGO_ID_ = ros::this_node::getNamespace();

    ROS_WARN_STREAM("[PID]: namespace: " << EGO_ID_);

    if (!nh.param<double>("params/k_p", K_P_, 0.0)) 
        ROS_WARN_STREAM("[PID] Did not load k_p");
    if (!nh.param<double>("params/k_i", K_I_, 0.0)) 
        ROS_WARN_STREAM("[PID] Did not load k_i");
    if (!nh.param<double>("params/k_d", K_D_, 0.0)) 
        ROS_WARN_STREAM("[PID] Did not load k_d");
    if (!nh.param<double>("params/max_v", MAX_VELOCITY_, 0.0)) 
        ROS_WARN_STREAM("[PID] Did not load max_v");
    if (!nh.param<double>("params/min_v", MIN_VELOCITY_, 0.0)) 
        ROS_WARN_STREAM("[PID] Did not load min_v");
    if (!nh.param<double>("params/desired_time_delta", DESIRED_TIME_DELTA_, 0.0)) 
        ROS_WARN_STREAM("[PID] Did not load desired_distance");

    if (!nh.param<std::string>("services/advertise", SERVICE_NAME_, "")) 
        ROS_WARN_STREAM("[PID] Did not load advertise");


    if (!nh.param<std::string>("topics/publish/error_topic", ERROR_TOPIC_, "")) 
        ROS_WARN_STREAM("[PID] Did not load advertise");
    if (!nh.param<std::string>("topics/publish/output_topic", OUTPUT_TOPIC_, "")) 
        ROS_WARN_STREAM("[PID] Did not load advertise");
    if (!nh.param<std::string>("topics/publish/distance_topic", DISTANCE_TOPIC_, "")) 
        ROS_WARN_STREAM("[PID] Did not load advertise");
    if (!nh.param<std::string>("topics/publish/set_point_topic", SET_POINT_TOPIC_, "")) 
        ROS_WARN_STREAM("[PID] Did not load advertise");

}


void setupPublisher(ros::NodeHandle& nh) {

    error_publisher_ = 
        nh.advertise<std_msgs::Float32>(
            ERROR_TOPIC_, 1, true
        );
    output_publisher_ = 
        nh.advertise<std_msgs::Float32>(
            OUTPUT_TOPIC_, 1, true
        );
    distance_publisher_ = 
        nh.advertise<std_msgs::Float32>(
            DISTANCE_TOPIC_, 1, true
        );
    set_point_publisher_ = 
        nh.advertise<std_msgs::Float32>(
            SET_POINT_TOPIC_, 1, true
        );
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pid_distance_service");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    loadParameter(private_nh);
    setupPublisher(private_nh);

    ros::ServiceServer pid_distance_service 
    = nh.advertiseService(SERVICE_NAME_, calculate_velocity);

    ROS_WARN("Ready pid_distance server!");

    // Wait for the service request
    ros::spin(); 

    return 0;
}
