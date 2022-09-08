# using stock Ubuntu 20.04 image
FROM ubuntu:focal

# no prompts from apt-get
ARG DEBIAN_FRONTEND=noninteractive

# use bash as primary shell for RUN commands
SHELL [ "/bin/bash", "-c" ]

# "fast" source list for my area
# NOTE: can be redundant or negatively affect download speed!
COPY sources.dallas.list /etc/apt/sources.list

# installing initial setup packages
RUN apt-get update && apt-get -y --no-install-recommends install \
    git \
    curl \
    wget \
    build-essential \
    cmake \
    lsb-release \
    python3-pip \
    python3-dev \
    gnupg2 \
    libxext6 \
    libx11-6 \
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    freeglut3-dev \
    net-tools \
    mesa-utils \
    mesa-utils-extra \
    locales \
    net-tools \
    software-properties-common \
    && apt-get -y autoremove \
    && apt-get clean

# installing CUDA 11.7.1
RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-keyring_1.0-1_all.deb && \
    dpkg -i cuda-keyring_1.0-1_all.deb && \
    apt-get update && apt-get -y --no-install-recommends install cuda && \
    rm -rf cuda-keyring_1.0-1_all.deb

# installing ROS1 Noetic Desktop Full
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    apt-get update && apt-get -y --no-install-recommends install \
    ros-noetic-desktop-full \
    ros-noetic-moveit \
    ros-noetic-moveit-resources \
    ros-noetic-moveit-visual-tools \
    ros-noetic-moveit-resources-prbt-moveit-config \
    ros-noetic-pilz-industrial-motion \
    ros-noetic-gazebo-ros \
    ros-noetic-gazebo-ros-control \
    ros-noetic-ros-control \
    ros-noetic-ros-controllers \
    ros-noetic-roboticsgroup-upatras-gazebo-plugins \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    python3-catkin-tools

# installing Gazebo 11
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' && \
    wget https://packages.osrfoundation.org/gazebo.key -O - | apt-key add - && \
    apt-get update && apt-get -y --no-install-recommends install gazebo11 libgazebo11-dev

# define catkin workspace
ENV HOME /root
ENV CATKIN_WS /root/catkin_ws
ENV CATKIN_SRC /root/catkin_ws/src

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV ROS_DISTRO noetic
ENV QT_X11_NO_MITSHM 1
ENV TERM xterm-256color

# setup catkin workspace
# https://wiki.ros.org/rosdep
RUN rosdep init && \
    rosdep update && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    mkdir -p ${CATKIN_SRC} && \
    cd ${CATKIN_WS} && \
    catkin init && \
    rosdep install --from-paths src --ignore-src -r -y && \
    catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release

# enable GPU supports
# https://wiki.ros.org/docker/Tutorials/Hardware%20Acceleration#nvidia-docker2
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics