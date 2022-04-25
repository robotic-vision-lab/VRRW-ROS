# using NVIDIA CUDA OpenGL image
FROM nvidia/cudagl:11.4.2-devel-ubuntu20.04

# no prompts from apt-get
ARG DEBIAN_FRONTEND=noninteractive

# use bash as primary shell for RUN commands
SHELL [ "/bin/bash", "-c" ]

# YOU MAY WANT TO ADD --no-install-recommends TO REDUCE IMAGE SIZE

# for slow downloading mirror (no idea why this happens sometimes)
RUN apt-get update && apt-get -y install python3-pip apt-utils && \
    python3 -m pip install --upgrade pip setuptools wheel testresources && \
    python3 -m pip install --upgrade apt-mirror-updater && \
    apt-mirror-updater -a

# installing initial setup packages
RUN apt-get update && apt-get -y install \
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
    && apt-get -y autoremove \
    && apt-get clean

# installing ROS1 Noetic Desktop Full
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    apt-get update && apt-get -y ros-noetic-desktop-full

# installing addition ROS packages
RUN apt-get update && apt-get -y install \
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
ENV QT_X11_NO_MITSHM 1
ENV TERM xterm-256color

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV ROS_DISTRO noetic

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