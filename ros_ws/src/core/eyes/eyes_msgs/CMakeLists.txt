cmake_minimum_required(VERSION 2.8.3)
project(eyes_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation  # Needed to generate custom message types
  roscpp
  rospy
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
)

add_message_files(  # Tell catkin which custom message files to include
  FILES
  Track.msg
  TrackArray.msg
  SegmentList.msg
  Segment.msg
  OdomScan.msg
  PoseWithScan.msg
  ObstacleList.msg
  Obstacle.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs # Needed to generate custom messages
)
include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${rospy_INCLUDE_DIRS}
)

