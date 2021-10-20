# Dockerfiles

These are the files you need to build Docker images, which is needed to spawn containers.

## Before You Begin

:warning: Make sure you have `nvidia-docker-2` installed before proceeding if you are using NVIDIA hardware acceleration. Check their [installation instruction](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker).

For more information, check out [Using Hardware Acceleration with Docker](http://wiki.ros.org/docker/Tutorials/Hardware%20Acceleration).

## Building Instructions

```bash
$ cd Robosim-Docker
$ docker build -t robosim:latest -f ./dockerfiles/Dockerfile.noetic .
```

This will build the image named `robosim` with the tag `latest` ready to use for backend ROS planning. For more information, check out [docker build](https://docs.docker.com/engine/reference/commandline/build/) documentation.

## Running Instructions

The given shell script in `scripts/run_robosim_container.sh` will handle spawning appropriate container. It optionally takes two parameters, with default values.

```bash
# Assuming terminal still at [some_path]/Robosim-Docker
# Usage: [sh or ./]run_robosim_container.sh [container_name] [ROS_src_path]
# ROS_src_path will be mapped to /root/catkin_ws/src inside the container

# default container_name: robosim-container; default_path:/home/rvl/Workspace/Unity-Projects/ROS-Unity-Sim
$ sh scripts/run_robosim_container.sh

# new name and some different path
$ sh scripts/run_robosim_container.sh [my-container] [my/ros/src/path]

# second run only need the name
# in another terminal
$ sh scripts/run_robosim_container.sh my-container
```

First run of the script will spawn the container in detached mode (curent terminal is not inside container), with subsequent runs will attach a bash shell inside the container. You will see your terminal prompt change. **Subsequent command only needs the correct name as the path has been mapped in the first run!**

```bash
# external shell
$

# container shell
/root/
> 
```