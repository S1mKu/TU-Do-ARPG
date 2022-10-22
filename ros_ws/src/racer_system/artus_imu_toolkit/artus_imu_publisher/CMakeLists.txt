cmake_minimum_required(VERSION 2.8.3)
project(artus_imu_publisher)

find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure)

generate_dynamic_reconfigure_options(cfg/artus_imu_sensors.cfg)

catkin_package(DEPENDS CATKIN DEPENDS dynamic_reconfigure)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY cfg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS
	nodes/artus_imu_9DOF_Razor_M0_publisher.py
	nodes/artus_imu_LSM6DSOX_LIS3MDL_publisher.py
	nodes/artus_imu_MPU9250_publisher.py
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/nodes
)
