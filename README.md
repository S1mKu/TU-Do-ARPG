# Autonomous Racing Software Stack and Simulation Enviroment

This repository contains software for 1/10th scale autonomous race cars to compete in the [F1/10 competition](http://f1tenth.org/). It is developed by the Autonomous Racing Project Group of [TU Dortmund](https://ls12-www.cs.tu-dortmund.de/daes/).

## Documentation

* For general information and documentation check out our [wiki page](https://gitlab.fachschaften.org/groups/f1tenth-ws21-22/-/wikis/home).
* For further information and documentation check the wiki from the previous group: [wiki page](https://github.com/arpg-sophisticated/ar-tu-do/wiki).

## Requirements

ROS Melodic (TODO: upgrade)
Ubuntu 18.04 (upgrade with ROS upgrade)

This repository also includes source code for the Gazebo simulation environment. However, it is currently not in use. We keep it just in case someone wants to adopt it.
See the [wiki page](https://github.com/arpg-sophisticated/ar-tu-do/wiki) of the previous group for more information. 

## Installation

See install_dependencies.sh and install all remaining dependencies with 'sudo apt-get install ros-melodic-[DEPENDENCY]`. 

## Running it

Navigate to the workspace of the project and remove the old build directory for a clean build:
```
cd ros_ws/
rm -dr build/ devel/
```

Source the ROS environment, e.g.:
```
source /opt/ros/melodic/setup.bash
```

Build the Gym simulation environment:
```
catkin_make -DCMAKE_BUILD_TYPE=Release --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so -DCATKIN_WHITELIST_PACKAGES="time_optimal_mpc_python;f1tenth_gym_ros"
```

Source the project for the first time:
```
source devel/setup.bash
```

Build the driving algorithms and utilities:
```
catkin_make -DCATKIN_WHITELIST_PACKAGES= -DCATKIN_BLACKLIST_PACKAGES="time_optimal_mpc_python;f1tenth_gym_ros"
```

Run the simulation (configure it via the parameters at the top of the launch file):
```
roslaunch launch/simulator/sim_artus_all.launch
```

![](doc/obstacle_detection_ego_full.gif)

## Hardware

Our car is based on a 1/10th scale RC car ([Traxxas Ford Fiesta](https://traxxas.com/products/models/electric/ford-fiesta-st-rally)) with these additions:

- CPU/GPU board ([NVIDIA Jetson](https://www.nvidia.com/object/jetson-tk1-embedded-dev-kit.html))
- motor controller ([FOCBOX](https://www.enertionboards.com/FOCBOX-foc-motor-speed-controller.html))
- LIDAR scanner ([Hokuyo UST-10LX](https://www.hokuyo-usa.com/products/scanning-laser-rangefinders/ust-10lx))
- an inertial measurement unit ([Invensense MPU-9250](https://www.invensense.com/products/motion-tracking/9-axis/mpu-9250/))
- Brushless DC motor (replaces the standard brushed motor)

## License

This project (excluding git submodules) is under MIT and GPLv3 dual licensed - see the [MIT.LICENSE](MIT.LICENSE) and [GPLv3.LICENSE](GPLv3.LICENSE) file for details.

