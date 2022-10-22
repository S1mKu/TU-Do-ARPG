#pragma once

#include <string>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_srvs/Trigger.h>

#include "eyes_msgs/SegmentList.h"
#include "eyes_msgs/ObstacleList.h"
#include "eyes_msgs/Obstacle.h"

#include "data_t.hpp"
#include "state_estimation.hpp"


#define DEFAULT_TOPIC_SEGMENTS "/lidar_segments"

class ObstacleTracking
{

    public:

        /*!
        * Constructor.
        * @param nodeHandle the ROS node handle.
        */
        ObstacleTracking(ros::NodeHandle& private_nh, ros::NodeHandle& nh);

    private:
        double euclidean_distance;

        double static_distance_threshold;

        StateEstimation stateEstim;

        std::string lidarFrame;
        std::string worldFrame;

        // std::vector<Obstacle> new_obstacles;
        std::vector<Obstacle> static_obstacles;
        std::vector<Obstacle> dynamic_obstacles;

        //! ROS topic names to subscribe to.
        std::string segmentsSubscriberTopic_;

        std::string pubObstaclesTopic_;

        //! ROS service server.
        ros::ServiceServer serviceServer_;

        ros::Time timestamp;

        //! ROS node handle.
        ros::NodeHandle& nodeHandle_;
        ros::NodeHandle& privateNodeHandle_;

        //! ROS topic subscriber.
        ros::Subscriber segmentsSubscriber_;

        ros::Publisher pub_rviz_obstacles;
        ros::Publisher pub_obstacles;
        ros::Publisher pub_waypoints;

        int maxStaticObstacleCount = 0;
        int maxDynamicObstacleCount = 0;

        int maxSegmentCount = 0;

        /*!
        * Reads and verifies the ROS parameters.
        * @return true if successful.
        */
        bool bReadParameters();
        void vSegmentsTopicCallback(const eyes_msgs::SegmentList::ConstPtr& segmentList);

        void removeInvalidated(std::vector<Obstacle> &obstacles);

        void visualizeSegements(const eyes_msgs::SegmentList& segments);
        void visualizeDynamicObstaclePath(Obstacle o);
        void visualizeObstacles(std::vector<Obstacle> &static_obstacles, std::vector<Obstacle> &dynamic_obstacles);

};
