#!/usr/bin/env bash

# Docker image and tag to use
IMAGE_NAME=robosim
IMAGE_TAG=noetic-desktop-focal

# get the full path of the script parent directory
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

# extension of usage of SCRIPT_DIR
# get the path of the included colcon_ws
HOST_CATKIN_WS=$( cd "${SCRIPT_DIR}/../catkin_ws/src" &> /dev/null && pwd )

# creating the docker container
docker run \
--tty \
--interactive \
--rm \
--privileged \
--gpus all \
--network host \
-e DISPLAY=$DISPLAY \
-v /tmp/.X11-unix:/tmp/.X11-unix \
-v ${HOST_CATKIN_WS}:/root/catkin_ws/src \
${IMAGE_NAME}:${IMAGE_TAG} \
bash
