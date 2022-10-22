cmake_minimum_required(VERSION 3.0.2)
project(pure_pursuit_service)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
  nav_msgs
  geometry_msgs
  hands_srv
  sensor_msgs
  raceline_service
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  nav_msgs
  hands_srv
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${catkin_LIBRARIES}
  ${roscpp_INCLUDE_DIRS}
)

add_executable(
  pure_pursuit_server src/pure_pursuit_server.cpp
  src/pure_pursuit_visualization.cpp
  )

# VERY IMPORTANT FOR BUILDING THE PROJECT!
add_dependencies(pure_pursuit_server hands_srv_generate_messages_cpp)

target_link_libraries(pure_pursuit_server
  ${catkin_LIBRARIES}
)

