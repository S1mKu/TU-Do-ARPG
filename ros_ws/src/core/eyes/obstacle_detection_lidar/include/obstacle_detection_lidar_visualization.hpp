#pragma one

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include "eyes_msgs/SegmentList.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/MarkerArray.h"

#include <cstdlib> 
#include <ctime>

// PCL specific includes
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "data.hpp"

class ScanFilterVisualization {

public:
    ScanFilterVisualization(ros::NodeHandle& nh);

    void visualizeScan(sensor_msgs::LaserScan scan);

    void visualizeSegements(eyes_msgs::SegmentList& segments);
    void visualizeFilteredScan(std::vector<beam_t> laser_beams);
    void visualizeAlignedBeams(std::vector<beam_t> laser_beams);
    void visualizeUnalignedBeams(std::vector<beam_t> laser_beams);

private:

    std::size_t maxSegmentCount = 0;
    std::size_t maxValidScanInds = 0;

    std::string LASER_FRAME_;
    std::string MAP_FRAME_;

    ros::NodeHandle node_handle_;
    tf::TransformListener listener_;

    void loadParameter();

    void setupPublisher();
    ros::Publisher visu_segment_publisher_;

};