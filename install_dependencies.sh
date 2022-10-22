#!/bin/bash

ROS_DISTRO=melodic
PATHROS="/opt/ros/melodic/setup.bash"

# Install dependencies for ws
sudo apt update && apt install -y python-pip  
sudo pip install autopep8

# Setup ws
wstool init
wstool up
source $PATHROS

# Install all ros dependencies
rosdep install -y --from-paths ros_ws/src --ignore-src --rosdistro ${ROS_DISTRO}

sudo apt-get install -y ros-melodic-ackermann-msgs ros-melodic-geometry2

[[ -n $SETUP_GIT ]] || read -p "Do you want to get/update the submodules (Y/n)? " -n 1 -r
printf "\n"

# if [[ $REPLY =~ ^[Yy]$ ]] || [ -z $REPLY ] || [[ -n $SETUP_GIT ]]; then
#     # Update sub-modules
#     git submodule init
#     git submodule update --recursive
#     printf "Check the branches now. They are maybe changed by this operation"
# fi

# Setup do-mpc dependencies
sudo bash -c 'cat << EOF >/etc/ros/rosdep/sources.list.d/artus-dependencies.yaml
python3-do-mpc:
  ubuntu:
    pip:
      packages: [do-mpc]
EOF'

sudo bash -c 'cat << EOF >/etc/ros/rosdep/sources.list.d/10-artus-dependencies.list 
yaml file:///etc/ros/rosdep/sources.list.d/artus-dependencies.yaml
EOF'

rosdep update
