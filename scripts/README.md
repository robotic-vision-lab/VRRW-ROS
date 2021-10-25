# Convenient Scripts

Collection of frequently used or referenced scripts for Docker, Bash, etc.

## initial_setup.sh

This script meant to setup the workspace, placing folders, files, and assign aliases approriately. **Currently not implemented**.

## run_robosim_container.sh

This script creates a container if given parameters are not found, otherwise attach to found container. Curently positional arguments are statically defined and need to be modified manually.

Reference the following `docker run` command to run the container. For more details, check out [docker run](https://docs.docker.com/engine/reference/run/) documentation

```bash
# creating the docker container
# see https://docs.docker.com/engine/reference/run/ for more details
docker run -it --name ${CONTAINER_NAME}            # shorthand --tty + --interactive \
--privileged                                       # extending all devices on host \
--network host                                     # use host network interface \
--shm-size 16G                                     # set /dev/shm size (small shm causes issues with CUDA ML stuff, like TF and Torch) \
--runtime nvidia                                   # nvidia-docker2 requirement \
-e "DISPLAY=${DISPLAY}"                            # specify display to spawn windows on \
-v "/tmp/.X11-unix:/tmp/.X11-unix:rw"              # map X11 into container with read/write permission \
-v "${HOST_CATKIN_PATH}:/root/catkin_ws/src:rw"    # map catkin workspace into container \
robosim:latest                                     # which image to use (default robosim:latest) \
bash                                               # spawn a bash when we're in the container \
```

## Internal Aliases

See [`.bash_profile`](../configs/.bash_aliases).