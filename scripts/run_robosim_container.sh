#!/usr/bin/env sh

# courtesy of https://github.com/NVlabs/Deep_Object_Pose/blob/master/docker/run_dope_docker.sh

# bash script folder; https://stackoverflow.com/a/3915420

SCRIPT_PATH="$(dirname "$0")"

CONFIGS_PATH="${SCRIPT_PATH}/../configs"

if [ -z "${CONTAINER_NAME}" ]; then
    CONTAINER_NAME=robosim-container
fi

if [ -z "${HOST_CATKIN_WS}" ]; then
    HOST_CATKIN_WS="${SCRIPT_PATH}/../catkin_ws/src"
fi

# get docker container ID if exists
ROBOSIM_ID=`docker ps -aqf "name=^/${CONTAINER_NAME}$"`

if [ -z "${ROBOSIM_ID}" ]; then
    # container DNE, creating one with either given name or default name (robosim-container)
    echo "Creating new ROBOSIM container in background named ${CONTAINER_NAME}"
    sleep 1

    # allow X Server window spawning
    xhost +local:root

    # creating the docker container
    # see https://docs.docker.com/engine/reference/run/ for more details
    docker run -t -d --name ${CONTAINER_NAME} --privileged --network=host --shm-size 16G --runtime nvidia -e "DISPLAY=${DISPLAY}" -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" -v "${HOST_CATKIN_WS}:/root/catkin_ws/src:rw" robosim:latest bash

    # add convenient aliases
    docker cp ${CONFIGS_PATH}/.bash_aliases ${CONTAINER_NAME}:/root/.bash_aliases
else
    echo "Found ROBOSIM container: ${CONTAINER_NAME}"
    # Check if the container is already running and start if necessary.
    if [ -z `docker ps -qf "name=^/${CONTAINER_NAME}$"` ]; then
        xhost +local:${ROBOSIM_ID}
        echo "${CONTAINER_NAME} container not running. Starting container..."
        docker start ${ROBOSIM_ID}
        docker exec -it ${ROBOSIM_ID} bash
    else
        xhost +local:${ROBOSIM_ID}
        echo "Attaching to running ${CONTAINER_NAME} container..."
        docker exec -it ${ROBOSIM_ID} bash
    fi
fi