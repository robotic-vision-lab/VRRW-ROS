#!/usr/bin/env bash

# TODO: check if Docker is installed
# TODO: check OS and Linux distribution to dynamically swap docker run and exec

# get the full path of the script parent directory
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

# extension of usage of SCRIPT_DIR
# get the path of the included colcon_ws
HOST_CATKIN_WS=$( cd "${SCRIPT_DIR}/../catkin_ws/src" &> /dev/null && pwd )

# name of the container to be created
CONTAINER_NAME=vr-demo-container

# Docker image and tag to use
IMAGE_NAME=rvl-vr
IMAGE_TAG=ros1-noetic-windows

# get docker container ID if exists
CONTAINER_ID=`docker ps -aqf "name=^/${CONTAINER_NAME}$"`

# if container with name not found
if [ -z "${CONTAINER_ID}" ]; then

    # creating the docker container
    docker run -t -d \
    --privileged \
    --name ${CONTAINER_NAME} \
    --gpus all \
    --shm-size 16G \
    --device /dev/dxg \
    -p 50000-50003:50000-50003 \
    -p 54321:54321 \
    -p 10000:10000 \
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

    # add convenient aliases
    docker cp ${SCRIPT_DIR}/.bash_aliases ${CONTAINER_NAME}:/root/.bash_aliases

else

    # Check if the container is already running and start if necessary.
    if [ -z `docker ps -qf "name=^/${CONTAINER_NAME}$"` ]; then
        docker start ${CONTAINER_ID}
    fi

    docker exec -it ${CONTAINER_ID} bash

fi