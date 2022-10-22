#include <ros/ros.h>
#include "control_command_handle.h"

#include <dynamic_reconfigure/server.h>
#include <control_command_handle/control_command_handle_Config.h>
using dc_server = dynamic_reconfigure::Server<control_command_handle::control_command_handle_Config>;

int main(int argc,  char* argv[])
{ 
    ROS_INFO("Node started");
    ros::init (argc, argv, "pure_pursuit");
    ros::NodeHandle public_nh;
    ros::NodeHandle private_nh("~");

    dc_server m_dyn_cfg_server;

    ros::Duration(3).sleep();

    ControlCommandHandle ControlCommandHandle(public_nh, private_nh, m_dyn_cfg_server);

    ros::spin();
    return 0;
}