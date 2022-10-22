#!/bin/bash

CATKIN_SHELL=bash
ROS_DISTRO=melodic
ARTUS_ROOT=$( cd "$(dirname "${BASH_SOURCE[0]}" )" && pwd)

source /opt/ros/$ROS_DISTRO/setup.bash

# check if devel folder exists
if [ -f "${ARTUS_ROOT}/devel/setup.bash" ]; then
    source "${ARTUS_ROOT}/devel/setup.bash"
else
    printf "You need to build first before you can source\n"
fi

# check ip config
[[ -f "${ARTUS_ROOT}/ipconfig" ]] && source "${ARTUS_ROOT}/ipconfig" || true