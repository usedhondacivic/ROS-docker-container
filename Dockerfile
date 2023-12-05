# Start with the ros noetic base image
FROM osrf/ros:noetic-desktop-full

# Install ros dependencies needed for CS 4750 (check README for the last semester this was updated)
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get -yq --force-yes install \
    sudo \
    dialog \
    less \
    x-window-system \
    mesa-utils \
    nano \
    curl \
    iproute2 \
    ros-noetic-dynamixel-sdk \
    ros-noetic-rospy \
    ros-noetic-dynamixel-sdk \
    ros-noetic-joint-state-publisher \
    ros-noetic-joint-state-publisher-gui \
    ros-noetic-robot-state-publisher \
    ros-noetic-rviz \
    ros-noetic-xacro \
    ros-noetic-controller-manager \
    ros-noetic-effort-controllers \
    ros-noetic-gazebo-ros \
    ros-noetic-gazebo-ros-control \
    ros-noetic-joint-state-controller \
    ros-noetic-joint-trajectory-controller \
    ros-noetic-moveit \
    ros-noetic-moveit-commander \
    ros-noetic-moveit-visual-tools \
    ros-noetic-rqt-plot \
    ros-noetic-moveit-simple-controller-manager \
    ros-noetic-moveit-fake-controller-manager \
    ros-noetic-moveit-planners-ompl \
    ros-noetic-moveit-ros-visualization \
    ros-noetic-moveit-setup-assistant \
    ros-noetic-dynamixel-workbench-toolbox \
    ros-noetic-rosbash \
    ros-noetic-tf-conversions \
    ros-noetic-fake-localization \
    ros-noetic-realsense2-camera \
    ros-noetic-realsense2-description \
    ros-noetic-moveit-resources-prbt-moveit-config \
    ros-noetic-move-base-msgs \
    ros-noetic-ackermann-msgs \
    ros-noetic-joy \
    ros-noetic-map-server \ 
    landscape-client \
    python3-catkin-tools \
    python3-rosdep \
    python3-tk \
    python3-pip \
    python3-dev \
    libcanberra-gtk-module

# Install python dependencies
RUN pip3 install modern_robotics imutils networkx cython

# Range_libc has to be built from source, install the python wrapper
COPY range_libc /home/range_libc
WORKDIR /home/range_libc/pywrapper
RUN python3 setup.py install

# Copy bashrc so that everything gets sourced properly
COPY bashrc =/home/noetic-dev/.bashrc
COPY bashrc /root/.bashrc
COPY bash_profile /home/noetic-dev/.bash_profile
COPY bash_profile /root/.bash_profile

# Build the workspaces
COPY interbotix_ws /root/noetic-dev/ros/interbotix_ws
WORKDIR /root/noetic-dev/ros/interbotix_ws
RUN  /bin/bash -c 'source /opt/ros/noetic/setup.bash && catkin build'
COPY dependencies_ws /root/noetic-dev/ros/dependencies_ws
WORKDIR /root/noetic-dev/ros/dependencies_ws
RUN  /bin/bash -c 'source /opt/ros/noetic/setup.bash && catkin build'
COPY homework_ws /root/noetic-dev/ros/homework_ws
WORKDIR /root/noetic-dev/ros/homework_ws
RUN  /bin/bash -c 'source /opt/ros/noetic/setup.bash && catkin build'

# Start in the homework_ws source directory 
WORKDIR /root/noetic-dev/ros/homework_ws/src/
