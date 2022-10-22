

#include <ros/ros.h>
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/PolygonStamped.h"
#include "nav_msgs/Path.h"

class PurePursuitVisualization {

public:
    PurePursuitVisualization(ros::NodeHandle& nh);

    void visualize(geometry_msgs::PolygonStamped polygon, const geometry_msgs::Point32 point);
    void look_ahead(const geometry_msgs::Point32 point);
    void followed_path(geometry_msgs::PolygonStamped polygon);

private:

    std::string MODEL_BASE_LINK_;
    std::string MAP_FRAME_;

    std::string VIZ_LH_TOPIC_;
    std::string VIZ_FP_TOPIC_;

    ros::NodeHandle node_handle_;

    void loadParameter();

    void setupPublisher();
    ros::Publisher followed_path_publisher_;
    ros::Publisher look_ahead_publisher_;

};