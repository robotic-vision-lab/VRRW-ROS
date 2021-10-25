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

The given shell script in `scripts/run_robosim_container.sh` will handle spawning appropriate container.

```bash
$ sh scripts/run_robosim_container.sh

# or

$ sh [path_to_repo]/scripts/run_robosim_container.sh
```

First run of the script will spawn the container in detached mode (curent terminal is not inside container), with subsequent runs will attach a bash shell inside the container. You will see your terminal prompt change.

```bash
# external shell
$

# container shell
/root/
> 
```