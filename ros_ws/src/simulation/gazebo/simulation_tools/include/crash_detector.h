#pragma once

#include <gazebo/gazebo.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <ros/ros.h>
#include <std_msgs/Empty.h>

constexpr const char* TOPIC_CRASH = "/crash";
constexpr const char* TOPIC_GAZEBO_SENSOR_WALLS =
    "/gazebo/racetrack/track/walls-collision-link/walls-contact-sensor/contacts";
constexpr const char* TOPIC_GAZEBO_SENSOR_DECORATION =
    "/gazebo/racetrack/track/decoration-collision-link/decoration-contact-sensor/contacts";
constexpr const char* TOPIC_GAZEBO_SENSOR_OBSTACLE =
    "/gazebo/racetrack/track/obstacle-collision-link/obstacle-contact-sensor/contacts";

/**
 * @brief ROS node that listens on a gazebo topic for collisions and publishes to a ROS topic
 */
class CrashDetector
{
    public:
    CrashDetector();

    private:
    ros::NodeHandle m_ros_node_handle;
    ros::Publisher m_crash_publisher;

    gazebo::transport::NodePtr m_gazebo_node;
    gazebo::transport::SubscriberPtr m_walls_sensor_subscriber;
    gazebo::transport::SubscriberPtr m_decoration_sensor_subscriber;
    gazebo::transport::SubscriberPtr m_obstacle_sensor_subscriber;

    void gazeboTopicCallback(ConstContactsPtr& message);
};
