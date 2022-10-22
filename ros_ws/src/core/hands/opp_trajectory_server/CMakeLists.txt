cmake_minimum_required(VERSION 3.0.2)
project(opp_trajectory_service)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
  nav_msgs
  eyes_msgs
  hands_msgs
  geometry_msgs
  hands_srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  eyes_msgs
  hands_msgs
  nav_msgs
  hands_srv
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${catkin_LIBRARIES}
  ${roscpp_INCLUDE_DIRS}
)

add_executable(opp_trajectory_server src/opp_trajectory_server.cpp)

# VERY IMPORTANT FOR BUILDING THE PROJECT!
add_dependencies(opp_trajectory_server hands_srv_generate_messages_cpp)

target_link_libraries(opp_trajectory_server ${catkin_LIBRARIES})
