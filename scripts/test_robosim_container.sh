xhost +local:root
docker run --rm -t -i --name robosim-test --privileged --network=host --shm-size 16G --runtime nvidia -e "DISPLAY=${DISPLAY}" -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" robosim:latest bash
