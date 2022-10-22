cmake_minimum_required(VERSION 2.8.3)
project(obstacle_detection_lidar)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_conversions
  pcl_ros
  std_msgs
  eyes_msgs
  drive_msgs
  sensor_msgs
  geometry_msgs
  laser_geometry
  nav_msgs
  f1tenth_gym_agent
  centerline_service
  cmake_modules REQUIRED
)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen REQUIRED)
# find_package(absl REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)


## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node 
  src/main.cpp
  src/grid_map.cpp
  src/obstacle_detection_lidar_node.cpp
  src/obstacle_detection_lidar_visualization.cpp
  src/background_substraction.cpp
  src/segmentation.cpp
  src/scan_matching.cpp
)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_dependencies(${PROJECT_NAME}_node eyes_msgs_generate_messages_cpp)

# add_dependencies(${PROJECT_NAME}_node hands_srv_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
