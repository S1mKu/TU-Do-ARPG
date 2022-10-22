cmake_minimum_required(VERSION 3.0.2)
project(hands_srv)

find_package(catkin REQUIRED COMPONENTS
  message_generation  # Needed to generate custom message types
  roscpp
  rospy
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  Opp_Trajectory.srv
  Pure_Pursuit.srv
  Gap_Follower.srv
  PID_Distance.srv
  Stanley_Controller.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs nav_msgs sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${rospy_INCLUDE_DIRS}
)
