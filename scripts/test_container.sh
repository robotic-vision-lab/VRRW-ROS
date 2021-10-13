#!/bin/bash

# https://stackoverflow.com/a/16496491

# usage() 
# { 
#     echo "Usage: $0 [-i <image_name>] [-t <image_tag>]" 1>&2;
#     echo "Available images are:";
#     echo "    mqt0029/robosim:";
#     echo "        bionic-melodic-cuda-base";
#     exit 1; 
# }

# while getopts ":i:t:" o; do
#     case "${o}" in
#         i)
#             i=${OPTARG}
#             ;;
#         t)
#             t=${OPTARG}
#             ;;
#         *)
#             usage
#             ;;
#     esac
# done
# shift $((OPTIND-1))

# if [ -z "${i}" ] || [ -z "${t}" ]; then
#     usage
# fi

# IMAGE_NAME="${i}"
# IMAGE_TAG="${t}"

IMAGE_NAME=robosim
IMAGE_TAG=noetic
CONTAINER_NAME=roboreal

echo "Starting temporary Docker container named ${CONTAINER_NAME} for ${IMAGE_NAME}:${IMAGE_TAG}";
sleep 1;

xhost +local:root

docker run -it --privileged --shm-size 16G \
--name ${CONTAINER_NAME} \
--runtime nvidia \
--network host \
-e "DISPLAY=$DISPLAY" \
-v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
-v "/home/rvl/Workspace/Unity-Projects/ROS-Unity-Sim:/root/catkin_ws/src:rw" \
${IMAGE_NAME}:${IMAGE_TAG} \
bash

# -e "DISPLAY=$DISPLAY" \
# -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
# -v "/home/rvl/Workspace/RVL/UR-Unity-ROS:/root/catkin_ws/src:rw" \
# --runtime nvidia \