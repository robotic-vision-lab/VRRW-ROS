#!/usr/bin/bash

# see https://stackoverflow.com/a/246128
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

# extension of usage of SCRIPT_DIR
HOST_COLCON_WS=$( cd "$( dirname -- "${BASH_SOURCE[0]}" )/../colcon_ws/src" &> /dev/null && pwd )

# name of the container to be created
CONTAINER_NAME=ros2-robosim-container

# Docker image and tag to use
IMAGE_NAME=robosim
IMAGE_TAG=humble-desktop-jammy-cuda

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
-v ${HOST_COLCON_WS}:/root/colcon_ws/src \
${IMAGE_NAME}:${IMAGE_TAG} \
bash