# build from stock Ubuntu 22.04 image
FROM ubuntu:22.04

# no prompts from apt-get
ARG DEBIAN_FRONTEND=noninteractive

# use bash as primary shell for RUN commands
SHELL [ "/bin/bash", "-c" ]

# [OPTIONAL] get an apt mirror instead of using stock archive.ubuntu.com
# for some reason, this is slow at my university...
COPY sources.list.dallas /etc/apt/sources.list

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
    mesa-utils \
    mesa-utils-extra \
    locales \
    net-tools \
    software-properties-common \
    && add-apt-repository universe \
    && apt-get -y autoremove \
    && apt-get clean

# setup locales
RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV LANG en_US.UTF-8
ENV LC_ALL C.UTF-8
ENV ROS_DISTRO humble
ENV COLCON_WS /root/colcon_ws
ENV COLCON_SRC /root/colcon_ws/src

# installing CUDA 11.7.1
RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.0-1_all.deb && \
    dpkg -i cuda-keyring_1.0-1_all.deb && \
    rm cuda-keyring_1.0-1_all.deb && \
    apt-get update && \
    apt-get -y --no-install-recommends install cuda

# upgrade MESA for Ignition Gazebo
RUN add-apt-repository ppa:kisak/kisak-mesa && \
    apt-get update && \
    apt-get -y full-upgrade

# installing ROS2 Humble Hawksbill
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update && \
    apt-get -y upgrade && \
    apt-get -y --no-install-recommends install \
    ros-${ROS_DISTRO}-desktop-full \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-gazebo-ros \
    ros-${ROS_DISTRO}-gazebo-ros2-control \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    python3-colcon-common-extensions \
    python3-colcon-argcomplete \
    python3-vcstool \
    python3-rosdep

# install classic Gazebo
RUN apt-get update && apt-get -y --no-install-recommends install gazebo

# pre-source setup file
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc && \
    echo "export _colcon_cd_root=/opt/ros/humble/" >> ~/.bashrc && \
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc

# setup empty colcon workspace
RUN mkdir -p ${COLCON_SRC} && \
    cd ${COLCON_WS} && \
    rosdep init && \
    rosdep update && \
    rosdep install --ignore-src --from-paths src -y -r && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    echo "source ${COLCON_WS}/install/setup.bash" >> ~/.bashrc

# NVIDIA access for WSL
ENV LD_LIBRARY_PATH=/usr/lib/wsl/lib

# ROS NVIDIA acceleration
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# misc.
ENV QT_X11_NO_MITSHM 1
ENV TERM xterm-256color