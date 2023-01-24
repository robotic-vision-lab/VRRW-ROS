# using OSRF ROS 1 Noetic image
FROM osrf/ros:noetic-desktop-full-focal

# no prompts from apt-get
ARG DEBIAN_FRONTEND=noninteractive

# use bash as primary shell for RUN commands
SHELL [ "/bin/bash", "-c" ]

# "fast" source list for my area
# NOTE: can be redundant or negatively affect download speed!
RUN cp /etc/apt/sources.list /etc/apt/sources.list.original && \
    sed -i -r 's,http://(.*).ubuntu.com,http://mirror.us-tx.kamatera.com,' /etc/apt/sources.list

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
    python-is-python3 \
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
    iputils-ping \
    netcat \
    software-properties-common \
    && apt-get update \
    && apt-get -y autoremove \
    && apt-get clean

# installing ROS1 Noetic Desktop Full
RUN apt-get update && apt-get -y --no-install-recommends install \
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
RUN rosdep update && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    mkdir -p ${CATKIN_SRC} && \
    cd ${CATKIN_WS} && \
    catkin init && \
    rosdep install --from-paths src --ignore-src -r -y && \
    catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release

# always pre sourcing at least the default setup.bash
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

# NVIDIA access for WSL
ENV LD_LIBRARY_PATH=/usr/lib/wsl/lib

# ROS NVIDIA acceleration, see http://wiki.ros.org/docker/Tutorials/Hardware%20Acceleration#nvidia-docker2
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics