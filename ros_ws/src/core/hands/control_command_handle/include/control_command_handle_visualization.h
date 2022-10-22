#pragma one

#include <ros/ros.h>

#include "eyes_msgs/SegmentList.h"
#include "geometry_msgs/Point32.h"
#include "visualization_msgs/MarkerArray.h"

class ControlCommandHandleVisualization {

public:
    ControlCommandHandleVisualization(ros::NodeHandle& nh);

    void visualize();
    void goalPoint(const geometry_msgs::Point32 point);

private:

    std::string MODEL_BASE_LINK_;
    std::string MAP_FRAME_;

    ros::NodeHandle node_handle_;

    void loadParameter();

    void setupPublisher();
    ros::Publisher visu_publisher_;

};