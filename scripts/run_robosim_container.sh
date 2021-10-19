#!/bin/bash

# courtesy of https://github.com/NVlabs/Deep_Object_Pose/blob/master/docker/run_dope_docker.sh

CONTAINER_NAME=$1

if [ -z "${CONTAINER_NAME}" ]; then
    CONTAINER_NAME=robosim-container
    # echo "No container name supplied, using ${CONTAINER_NAME}"
fi

# specify catkin src folder from host machine
HOST_CATKIN_PATH=$2

if [ -z "${HOST_CATKIN_PATH}" ]; then
    HOST_CATKIN_PATH=/home/rvl/Workspace/Unity-Projects/ROS-Unity-Sim
    # while true; do
    #     read -p "No ROS src folder set, using default path = ${HOST_CATKIN_PATH}? [YyNn] " yn
    #     case $yn in
    #         [Yy]* ) echo "HOST_CATKIN_PATH=${HOST_CATKIN_PATH}"; break;;
    #         [Nn]* ) exit;;
    #         * ) echo "Please answer yes or no.";;
    #     esac
    # done
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
    docker run -t -d --rm --name ${CONTAINER_NAME} --privileged --network=host --shm-size 16G --runtime nvidia -e "DISPLAY=${DISPLAY}" -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" -v "${HOST_CATKIN_PATH}:/root/catkin_ws/src:rw" robosim:latest bash
    docker exec ${CONTAINER_NAME} touch /root/.bash_eternal_history 
    docker cp $(dirname "$BASH_SOURCE")/configs/.bash_aliases ${CONTAINER_NAME}:/root/.bash_aliases
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

# # creating the docker container
# # see https://docs.docker.com/engine/reference/run/ for more details
# docker run -it --name ${CONTAINER_NAME}            # shorthand --tty + --interactive \
# --privileged                                       # extending all devices on host \
# --network host                                     # use host network interface \
# --shm-size 16G                                     # set /dev/shm size (small shm causes issues with CUDA ML stuff, like TF and Torch) \
# --runtime nvidia                                   # nvidia-docker2 requirement \
# -e "DISPLAY=${DISPLAY}"                            # specify display to spawn windows on \
# -v "/tmp/.X11-unix:/tmp/.X11-unix:rw"              # map X11 into container with read/write permission \
# -v "${HOST_CATKIN_PATH}:/root/catkin_ws/src:rw"    # map catkin workspace into container \
# robosim:latest                                     # which image to use (default robosim:latest) \
# bash                                               # spawn a bash when we're in the container \