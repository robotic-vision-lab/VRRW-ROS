SCRIPT_PATH="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"
CONFIGS_PATH="${SCRIPT_PATH}/../configs"
HOST_CATKIN_WS="${SCRIPT_PATH}/../catkin_ws/src"

xhost +local:root
docker run --rm -t -i --name robosim-test --privileged --network=host --shm-size 16G --runtime nvidia -e "DISPLAY=${DISPLAY}" -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" -v "${HOST_CATKIN_WS}:/root/catkin_ws/src:rw" robosim:latest bash
