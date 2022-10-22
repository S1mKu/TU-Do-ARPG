# Base image
FROM ros:melodic-robot-bionic

# Update apt repo and pip2, and install python3, pip3
RUN apt-get update --fix-missing && \
    apt-get install -y python-pip \
                       python3-dev \
                       python3-pip \
                       python3.8 \
                       python3.8-dev \
                       python3.8-distutils \
                       python3.8-venv \
                       python3-catkin-pkg-modules \
                       python3-rospkg-modules \
                       python3-empy

# Install apt dependencies, add your apt dependencies to this list
RUN apt-get install -y git \
                       cmake \
                       ros-melodic-ackermann-msgs \
		       ros-melodic-tf2-geometry-msgs \
		       ros-melodic-serial \
                       ros-melodic-genpy python-catkin-tools libsdl2-dev netcat ros-melodic-depthimage-to-laserscan
RUN apt-get install -y libignition-math2-dev nmap net-tools
RUN apt-get install -y python-rosinstall python-rosinstall-generator python-wstool build-essential python-rosdep

RUN apt-get install -y libsdl-image1.2-dev libsdl-ttf2.0-dev libsdl1.2-dev

# Upgrade pip
RUN pip install --upgrade pip

# Install pip dependencies, add your pip dependencies to this list
RUN pip install numpy==1.16.0 \
                scipy==1.2.0 \
                pyyaml \
		Cython
RUN pip3 install numpy==1.16.0 \
                  scipy==1.2.0 \
                  pyyaml \
		  Cython


# Creating a catkin workspace
RUN mkdir -p /catkin_ws

# Clone or copy over your source code

# Copying
 COPY ./ros_ws /catkin_ws

# Cloning
# RUN cd /catkin_ws/src/ && \
#     git clone https://github.com/your_user/your_repo.git

# Building your ROS packages
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash; cd catkin_ws; catkin_make; source devel/setup.bash; wstool init; wstool set -y src/geometry2 --git https://github.com/ros/geometry2 -v 0.6.5; wstool up; rosdep install --from-paths src --ignore-src -y -r;catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so; export ROS_MASTER_URI=http://172.17.0.1:11311"

# Uncomment set the shell to bash to provide the "source" command
SHELL ["/bin/bash", "-c"] 

# Setting entry point command that runs when the container is brought up
CMD source /catkin_ws/devel/setup.bash; roslaunch /catkin_ws/launch/simulator/sim_artus_agent_competition.launch;
