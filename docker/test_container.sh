#!/usr/bin/env bash

# Docker image and tag to use
IMAGE_NAME=robosim
IMAGE_TAG=noetic-desktop-focal-cuda

# get the full path of the script parent directory
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

# extension of usage of SCRIPT_DIR
# get the path of the included colcon_ws
HOST_CATKIN_WS=$( cd "${SCRIPT_DIR}/../catkin_ws/src" &> /dev/null && pwd )

# creating the docker container
docker run --rm -i -t \
--privileged \
--gpus all \
--shm-size 16G \
--device /dev/dxg \
-e DISPLAY=$DISPLAY \
-e WAYLAND_DISPLAY=$WAYLAND_DISPLAY \
-e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
-e PULSE_SERVER=$PULSE_SERVER \
-v /usr/lib/wsl:/usr/lib/wsl \
-v /tmp/.X11-unix:/tmp/.X11-unix \
-v /mnt/wslg:/mnt/wslg \
-v ${HOST_CATKIN_WS}:/root/catkin_ws/src \
${IMAGE_NAME}:${IMAGE_TAG} \
bash