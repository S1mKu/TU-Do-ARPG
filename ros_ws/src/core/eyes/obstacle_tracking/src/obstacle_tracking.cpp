#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "data_association.hpp"
#include "util.hpp"

#include "obstacle_tracking.hpp"


#include <chrono>


ObstacleTracking::ObstacleTracking(ros::NodeHandle& private_nh, ros::NodeHandle& nh)
    : privateNodeHandle_(private_nh), nodeHandle_(nh), stateEstim(private_nh)
{
    if (!bReadParameters())
    {
        ROS_ERROR("Could not read parameters.");
        ros::requestShutdown();
    }

    timestamp = ros::Time::now();

    segmentsSubscriber_ = nodeHandle_.subscribe(segmentsSubscriberTopic_, 10, &ObstacleTracking::vSegmentsTopicCallback, this);

    // RViz publishers
    pub_rviz_obstacles = privateNodeHandle_.advertise<visualization_msgs::Marker>("rviz_obstacles", 10);
    pub_waypoints = privateNodeHandle_.advertise<visualization_msgs::Marker>("rviz_waypoints", 5);

    pub_obstacles = nodeHandle_.advertise<eyes_msgs::ObstacleList>(pubObstaclesTopic_, 10);

    ROS_INFO("Successfully launched node ObstacleTracking.");
}

bool ObstacleTracking::bReadParameters()
{
    std::string ns = ros::this_node::getNamespace();
    std::string frame_prefix = "opp_racecar";

    if (ns == "/a1")
    {
        frame_prefix = "ego_racecar";
    }

    // use global namespace (including group namespace from launch file)
    if (!privateNodeHandle_.param<std::string>("topics/subscribe/final_filtered_scan_topic", segmentsSubscriberTopic_, "obstacle_detection/segments"))
        ROS_WARN_STREAM("Did not load topics/subscribe/final_filtered_scan_topic.");
    if (!privateNodeHandle_.param<std::string>("topics/publish/obstacle_topic", pubObstaclesTopic_, "obstacle_tracking/obstacles"))
        ROS_WARN_STREAM("Did not load topics/publish/obstacle_topic.");

    if (!privateNodeHandle_.param<std::string>("tf/frames/world_frame", worldFrame, std::string("map")))
        ROS_WARN_STREAM("Did not load tf/frames/world_frame.");
    if (!privateNodeHandle_.param<std::string>("tf/frames/car_frame", lidarFrame, std::string("laser_model")))
        ROS_WARN_STREAM("Did not load tf/frames/car_frame.");

    if (!privateNodeHandle_.param<double>("algorithms/data_association/euclidean_distance_threshold", static_distance_threshold, 4.0))
        ROS_WARN_STREAM("Did not load algorithms/data_association/euclidean_distance_threshold.");

    lidarFrame = frame_prefix + lidarFrame;

    return true;
}

void ObstacleTracking::removeInvalidated(std::vector<Obstacle> &obstacles)
{
    // remove invalidated obstacles
    for (unsigned int i = 0; i < obstacles.size(); i++)
    {
        Obstacle o = obstacles.at(i);
        if (o.invalidated)
        {
            obstacles.erase(obstacles.begin() + i);
            i--;
        }
    }
}

void ObstacleTracking::visualizeDynamicObstaclePath(Obstacle o)
{
    int pos_count = 0;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "waypoint_arrows";
    marker.action = visualization_msgs::Marker::ADD;      
    marker.type = visualization_msgs::Marker::ARROW;

    // marker.id = i;

    // arrows are red
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    marker.scale.x = 0.2;    // arrow length
    marker.scale.y = 0.04;    // arrow width
    marker.scale.z = 0.1;   // arrow height

    for (int i = 0; i < o.position_backlog.size(); i++)
    {
        Point p = o.position_backlog.at(i);
        double theta = o.theta_backlog.at(i);

        tf2::Quaternion quat_theta;
        quat_theta.setRPY(0, 0, theta);
        marker.pose.orientation = tf2::toMsg(quat_theta);
        marker.pose.position.x  = p.first;
        marker.pose.position.y  = p.second;
        marker.pose.position.z  = 0;

        marker.id = pos_count++;
        pub_rviz_obstacles.publish(marker);
    }
}

void ObstacleTracking::visualizeSegements(const eyes_msgs::SegmentList& segments)
{
    if (segments.segments.size() > maxSegmentCount)
    {
        maxSegmentCount = segments.segments.size();
    }

    for (int i = 0; i < maxSegmentCount; i++)
    {
        visualization_msgs::Marker line_strip;

        line_strip.header.frame_id = "map";
        line_strip.header.stamp = ros::Time::now();
        line_strip.ns = "incoming_segments";
        line_strip.action = visualization_msgs::Marker::ADD;
        line_strip.pose.orientation.w = 1.0;

        line_strip.id = i;
        
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;

        // Line strip is green
        line_strip.color.r = 0.0;
        line_strip.color.g = 1.0;
        line_strip.color.b = 0.0;
        line_strip.color.a = 1.0;

        line_strip.scale.x = 0.1;

        if (i < segments.segments.size()) {
            for (auto &point: segments.segments[i].points) {
                geometry_msgs::Point p;
                p.x    = point.x;
                p.y    = point.y;

                line_strip.points.push_back(point);
            }
        }
        else
        {
            line_strip.color.a = 0.0;
            geometry_msgs::Point p;
            p.x = 0.0;
            p.y = 0.0;

            // at least two points are required
            line_strip.points.push_back(p);
            line_strip.points.push_back(p);
        }

        pub_rviz_obstacles.publish(line_strip);
    }
}

void ObstacleTracking::visualizeObstacles(std::vector<Obstacle> &static_obstacles, std::vector<Obstacle> &dynamic_obstacles)
{
    /**
     * Visualization dynamic obstacles
     * 
     */
    visualization_msgs::Marker markers;

    markers.header.frame_id = "map";
    markers.header.stamp = ros::Time::now();
    markers.ns = "dynamic_obstacles";
    markers.action = visualization_msgs::Marker::ADD;
    markers.pose.orientation.w = 1.0;
    markers.id = 0;
    markers.type = visualization_msgs::Marker::LINE_STRIP;

    // points are red
    markers.color.r = 1.0;
    markers.color.g = 0.0;
    markers.color.b = 0.0;
    markers.color.a = 1.0;
    markers.scale.x = 0.1;

    // just for visualization purposes
    if (dynamic_obstacles.size() > maxDynamicObstacleCount)
    {
        maxDynamicObstacleCount = dynamic_obstacles.size();
    }

    for (unsigned int i = 0; i < maxDynamicObstacleCount; i++)
    {
        if (i < dynamic_obstacles.size())
        {
            Obstacle o = dynamic_obstacles.at(i);

            for (Point seg_p : o.segment_backlog.back().points)
            {
                geometry_msgs::Point geom_p;
                geom_p.x = seg_p.first;
                geom_p.y = seg_p.second;
                markers.points.push_back(geom_p);
            }
        }
        else
        {
            markers.color.a = 0.001;
            geometry_msgs::Point p;
            p.x = 0.0;
            p.y = 0.0;

            // at least two points are required
            markers.points.push_back(p);
            markers.points.push_back(p);
        }

        markers.id = i;
        pub_rviz_obstacles.publish(markers);

        // clear markers.points for reuse
        markers.points.clear();
    }


    /**
     * Visualization static obstacles
     * 
     */
    markers.header.frame_id = "map";
    markers.header.stamp = ros::Time::now();
    markers.ns = "static_obstacles";
    markers.action = visualization_msgs::Marker::ADD;
    markers.pose.orientation.w = 1.0;
    markers.id = 0;
    markers.type = visualization_msgs::Marker::LINE_STRIP;

    // points are blue
    markers.color.r = 0.0;
    markers.color.g = 0.0;
    markers.color.b = 1.0;
    markers.color.a = 1.0;
    markers.scale.x = 0.1;

    if (static_obstacles.size() > maxStaticObstacleCount)
    {
        maxStaticObstacleCount = static_obstacles.size();
    }

    for (unsigned int i = 0; i < maxStaticObstacleCount; i++)
    {
        if (i < static_obstacles.size())
        {
            Obstacle o = static_obstacles.at(i);

            for (Point seg_p : o.segment_backlog.back().points)
            {
                geometry_msgs::Point geom_p;
                geom_p.x = seg_p.first;
                geom_p.y = seg_p.second;
                markers.points.push_back(geom_p);
            }
        }
        else
        {
            markers.color.a = 0.001;
            geometry_msgs::Point p;
            p.x = 0.0;
            p.y = 0.0;

            // at least two points are required
            markers.points.push_back(p);
            markers.points.push_back(p);
        }

        markers.id = i;
        pub_rviz_obstacles.publish(markers);

        // clear markers.points for reuse
        markers.points.clear();
    }

    markers.header.frame_id = "map";
    markers.header.stamp = ros::Time::now();
    markers.ns = "points";
    markers.action = visualization_msgs::Marker::ADD;
    markers.pose.orientation.w = 1.0;
    markers.id = 0;
    markers.type = visualization_msgs::Marker::POINTS;

    // points are blue
    markers.color.r = 0.0;
    markers.color.g = 0.0;
    markers.color.b = 1.0;
    markers.color.a = 1.0;
    markers.scale.x = 0.1;

    for (Obstacle o : static_obstacles)
    {
        for (Point seg_p : o.segment_backlog.back().points)
        {
            geometry_msgs::Point geom_p;
            geom_p.x = seg_p.first;
            geom_p.y = seg_p.second;
            markers.points.push_back(geom_p);
        }
    }
    
    for (Obstacle o : dynamic_obstacles)
    {
        for (Point seg_p : o.segment_backlog.back().points)
        {
            geometry_msgs::Point geom_p;
            geom_p.x = seg_p.first;
            geom_p.y = seg_p.second;
            markers.points.push_back(geom_p);
        }
    }

    if (markers.points.size() > 0)
    {
        pub_rviz_obstacles.publish(markers);
    }
}

void ObstacleTracking::vSegmentsTopicCallback(const eyes_msgs::SegmentList::ConstPtr& segmentList)
{
    auto t1 = std::chrono::high_resolution_clock::now();

    visualizeSegements(*segmentList);

    int len_backlog = 20;
    timestamp = segmentList->header.stamp;
    std::vector<segment_t> segments;
    int count = 0;

    /**
     * "convert" SegmentList message to vector of PointList
     * 
     * only accept segments smaller or equal to the diameter of the racecar (which should be the only dynamic obstacle for now)
     * 
     * reject any other segments
     * 
     */
    for (int i = 0; i < segmentList->segments.size(); i++)
    {
        eyes_msgs::Segment segment_msg = segmentList->segments[i];
        segment_t segment;
        segment.invalidated = false;

        // each segement should contain more than two points so there should always be 
        // a distinct starting and endpoint
        geometry_msgs::Point start = segment_msg.points.front();
        geometry_msgs::Point end = segment_msg.points.back();

        double diameter = std::sqrt(std::pow(start.x - end.x, 2.0) + std::pow(start.y - end.y, 2.0));

        // apparently the threshold has to be much higer
        if (diameter <= 0.7) // exact 0.387714173 but give it some space
        {
            for (int j = 0; j < segment_msg.points.size(); j++)
            {
                double x = segment_msg.points[j].x;
                double y = segment_msg.points[j].y;
                segment.points.push_back(Point(x, y));

                count++;
            }

            segments.push_back(segment);
        }
    }

    /**
     * Associate static obstacles with segments [NOTE: the direction static obstacle -> segment counts]
     * 
     */
    std::vector<association_t> static_associations = associate_static_obstacles(static_obstacles, segments);

    std::vector<Obstacle *> leftover_static_obstacles;
    std::vector<Obstacle> leftover_obstacles;
    int static_obs = 0;

    int i = 0;
    for (association_t association : static_associations)
    {
        Obstacle *o = &static_obstacles.at(association.obstacle_index);

        if (association.obstacle_unassociated)
        {
            o->leftover_index = i;
            o->invalidated = true;
            leftover_static_obstacles.push_back(o);
            leftover_obstacles.push_back(*o);
        }
        else
        {
            segment_t *s = &segments.at(association.segment_index);

            if (association.distance < static_distance_threshold)
            {
                s->invalidated = true;
                static_obs++;
            }
            else
            {
                o->leftover_index = i;
                o->invalidated = true;
                leftover_static_obstacles.push_back(o);
                leftover_obstacles.push_back(*o);
            }
        }

        i++;
    }

    /**
     * Associate dynamic obstacles with segments [NOTE: the direction dynamic obstacle -> segment counts]
     * 
     */
    std::vector<association_t> associations = associate_dynamic_obstacles(dynamic_obstacles, segments);

    int obs = 0;

    i = 0;
    for (association_t association : associations)
    {
        Obstacle *o = &dynamic_obstacles.at(association.obstacle_index);

        if (association.obstacle_unassociated)
        {
            // mark unassociated dynamic obstacle for removal
            o->invalidated = true;

            // or should we keep the obstacle for a certain amount of time and "reactivate it"
        }
        else
        {
            segment_t *s = &segments.at(association.segment_index);
            s->invalidated = true; // mark segment as used/associated to make it unavailable for later associations or as new obstacle
            obs++; // increase counter for associated obstacles

            if (association.distance >= 0.25) // obstacle should have moved a small amount otherwise do not update now
            {
                double dt = (segmentList->header.stamp - o->timestamp).toSec();
                o->timestamp = timestamp;

                // update the obstacle and remove its associated segment from the list
                // o->segment_backlog.push_back(*s);
                stateEstim.updateObstacle(*o, *s, dt);
            }
            else
            {
                //TODO decide what to do - option would be to just do nothing
            }
        }

        i++;
    }

    /**
     * Associate leftover static obstacles with segments [NOTE: the direction dynamic obstacle -> segment counts]
     * 
     * here we determine whether a static obstacle moved and became dynamic
     * 
     */
    std::vector<association_t> leftover_associations = associate_dynamic_obstacles(leftover_obstacles, segments);

    int leftover_obs = 0;

    for (association_t association : leftover_associations)
    {
        Obstacle *o = leftover_static_obstacles.at(association.obstacle_index);

        if (association.obstacle_unassociated)
        {
            // mark unassociated dynamic obstacle for removal
            o->invalidated = true;

            // or should we keep the obstacle for a certain amount of time and "reactivate it"?
        }
        else
        {
            segment_t *s = &segments.at(association.segment_index);
            s->invalidated = true; // mark segment as used/associated to make it unavailable for later associations or as new obstacle
            obs++; // increase counter for associated obstacles

            segment_t segment = segments.at(association.segment_index);
            Point position = mean(segment.points);
            double theta = segmentList->observer_theta;

            //TODO fix type argument STATIC, ...
            Obstacle o(segment, position, theta, len_backlog, STATIC);

            dynamic_obstacles.push_back(o);

            // if (association.distance >= 0.25) // obstacle should have moved a small amount otherwise do not update now
            // {
            //     double dt = (segmentList->header.stamp - o->timestamp).toSec();
            //     o->timestamp = timestamp;

            //     // update the obstacle and remove its associated segment from the list
            //     // o->segment_backlog.push_back(*s);
            //     stateEstim.updateObstacle(*o, *s, dt);
            // }
            // else
            // {
            //     //TODO decide what to do - option would be to just do nothing
            // }
        }
    }

    /**
     * count invalidated obstacles for logging
     * 
     */
    int rem_obs = 0;
    for (Obstacle o : static_obstacles)
    {
        if (o.invalidated)
            rem_obs++;
    }

    for (Obstacle o : dynamic_obstacles)
    {
        if (o.invalidated)
            rem_obs++;
    }

    /**
     * remove invalidated obstacles and segments
     * 
     */
    removeInvalidated(static_obstacles);
    removeInvalidated(dynamic_obstacles);

    /**
     * Initialize new obstacles for every remaining segment
     * 
     * For now we only work with dynamic obstacles
     * 
     */
    int new_obs = 0;

    for (segment_t segment : segments)
    {
        if (!segment.invalidated)
        {
            Point position = mean(segment.points);
            double theta = segmentList->observer_theta;

            //TODO fix type argument STATIC, ...
            Obstacle o(segment, position, theta, len_backlog, STATIC);

            // dynamic_obstacles.push_back(o);
            static_obstacles.push_back(o);
            new_obs++;
        }
    }

    /**
     * publish obstacle list message
     * 
     */
    eyes_msgs::ObstacleList obstacle_list_msg;
    obstacle_list_msg.header = segmentList->header;

    for (Obstacle o : static_obstacles)
    {
        eyes_msgs::Obstacle obstacle_msg;
        obstacle_msg.velocity = 0.0;
        // obstacle_msg.is_dynamic = false;

        for (Point seg_p : o.segment_backlog.back().points)
        {
            geometry_msgs::Point geom_p;
            geom_p.x = seg_p.first;
            geom_p.y = seg_p.second;
            obstacle_msg.segment.points.push_back(geom_p);
        }

        obstacle_list_msg.obstacles.push_back(obstacle_msg);
    }

    for (Obstacle o : dynamic_obstacles)
    {
        eyes_msgs::Obstacle obstacle_msg;
        // obstacle_msg.is_dynamic = true;
        obstacle_msg.velocity = o.velocity;

        for (int ind = o.theta_backlog.size() - 1; ind >= 0; ind--)
        {
            obstacle_msg.theta.push_back(o.theta_backlog.at(ind));
        }

        for (int ind = o.position_backlog.size() - 1; ind >= 0; ind--)
        {
            obstacle_msg.x.push_back(o.position_backlog.at(ind).first);
            obstacle_msg.y.push_back(o.position_backlog.at(ind).second);
        }

        for (Point seg_p : o.segment_backlog.back().points)
        {
            geometry_msgs::Point geom_p;
            geom_p.x = seg_p.first;
            geom_p.y = seg_p.second;
            obstacle_msg.segment.points.push_back(geom_p);
        }

        obstacle_list_msg.obstacles.push_back(obstacle_msg);

        visualizeDynamicObstaclePath(o);
    }

    pub_obstacles.publish(obstacle_list_msg);

    /**
     * final wrap-up
     * 
     */
    visualizeObstacles(static_obstacles, dynamic_obstacles);

    auto t2 = std::chrono::high_resolution_clock::now();
    ROS_DEBUG_STREAM_DELAYED_THROTTLE_NAMED(
        10, 
        ros::this_node::getNamespace() + "_obstacle_tracking_state", 
        "static: " << static_obstacles.size() 
        <<  "dynamic: " << dynamic_obstacles.size() 
        << "  time: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() << "[ms]"
    );
}
