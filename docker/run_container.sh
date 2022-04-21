#!/usr/bin/env bash

# courtesy of https://github.com/NVlabs/Deep_Object_Pose/blob/master/docker/run_dope_docker.sh

# get the script parent directory
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

# location of catkin_ws on host machine in relation to script
HOST_CATKIN_WS=$( cd "$( dirname -- "${BASH_SOURCE[0]}" )/../catkin_ws/src" >/dev/null 2>&1 ; pwd -P )

# name of the container to be created
CONTAINER_NAME=unity-robosim-container

# Docker image and tag to use
IMAGE_NAME=unity-robosim
IMAGE_TAG=latest

# get docker container ID if exists
CONTAINER_ID=`docker ps -aqf "name=^/${CONTAINER_NAME}$"`

# if container with name not found
if [ -z "${CONTAINER_ID}" ]; then
    # creating the docker container
    # you may also want the --privileged flag instead of --device to access more devices
    docker run -t -d \
    --name ${CONTAINER_NAME} \
    --device /dev/dxg \
    --runtime nvidia \
    --gpus all \
    --shm-size 16G \
    --network host \
    -e "WAYLAND_DISPLAY=$WAYLAND_DISPLAY" \
    -e "XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR" \
    -e "PULSE_SERVER=$PULSE_SERVER" \
    -e "DISPLAY=$DISPLAY" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix" \
    -v "/mnt/wslg:/mnt/wslg" \
    -v "/usr/lib/wsl:/usr/lib/wsl" \
    -v "${HOST_CATKIN_WS}:/root/catkin_ws/src" \
    ${IMAGE_NAME}:${IMAGE_TAG} \
    bash

    # add convenient aliases
    docker cp ${SCRIPT_DIR}/.bash_aliases ${CONTAINER_NAME}:/root/.bash_aliases
else
    # allow X Server access
    xhost +local:`docker inspect --format='{{ .Config.Hostname }}' ${CONTAINER_ID}`

    # Check if the container is already running and start if necessary.
    if [ -z `docker ps -qf "name=^/${CONTAINER_NAME}$"` ]; then
        echo "${CONTAINER_NAME} container not running. Starting container..."
        docker start ${CONTAINER_ID}
    else
        echo "Attaching to running ${CONTAINER_NAME} container..."
    fi
    docker exec -it ${CONTAINER_ID} bash

    # remove previous access
    xhost -local:`docker inspect --format='{{ .Config.Hostname }}' ${CONTAINER_ID}`
fi