# Unity Robotics ROS-Side API

This repository contains the ROS-side API to communicate and control simulated or real UR5e robots
through Unity UI interface.

## Quick Start

1. Clone this repository and `cd` into it.

```console
user@DESKTOP:~$ git clone https://github.com/robotic-vision-lab/Unity-Robosim.git
user@DESKTOP:~$ cd Unity-Robosim
```

2. Build or pull the docker image

```console
user@DESKTOP:~$ cd docker
user@DESKTOP:~$ ./build_image.sh
```

3. Run the docker container

A convenient script is provided to run the docker container. The script will:

- Create a container if it does not exists.
- If the container exists, it will start the container, and attach to it.
- Mount the `catkin_ws` directory to the container.
- Expose sufficient ports and devices for rendering and ROS communication.

Subsequent re-run of the script will just attach to the existing container.

```console
user@DESKTOP:~$ ./run_container.sh
```

Several convenient aliases has also been defined in the container, and will be listed as user attach
to container.

```
user@DESKTOP:~$ ./run_container.sh
non-network local connections being added to access control list
predefined aliases: src_ros, run_rosdep, run_catkin, rebuild_catkin, kill_gazebo, kill_rviz

root ~/catkin_ws
>
```

4. Install dependencies and build catkin workspace

```console
root ~/catkin_ws
> run_rosdep

[...]

root ~/catkin_ws
> run_catkin

[...]

root ~/catkin_ws
> src_ros
```

> 🗒️ **In order for autocomplete to find all ROS commands and packages, for every new terminal or
rebuilding packages, you must run `src_ros` to source the ROS environment.**

5. Run the simulation or real robot control backend

```console
root ~/catkin_ws
> roslaunch rvl_unity_communicator start_backend.launch
```

or

```console
root ~/catkin_ws
> roslaunch rvl_unity_communicator start_real_robot.launch
```

> 🗒️ **Remember to modify the `start_real_robot.launch` file to match your robot's and control computer's IP addresses.**

6. Start Unity-side Application Package

Refer to the [Unity VR Robotic Workspace](https://github.com/robotic-vision-lab/Unity-VR-Robotic-Workspace.git) repository for more information.

## Troubleshooting

In many cases, the most common issues are:

1. Either the ROS-side, Unity-side, or physical robot (if applicable), does not have proper
   permission to communicate through LAN. To ensure that all devices are able to communicate with
   each other, a simple ping test might be suffice:

```console
user@DESKTOP:~$ ping <IP_ADDRESS>
```

Raw IP address i.e. `192.168.___.___` is preferred over hostname i.e. `localhost` or `robot.local`,
however, I have not experienced any issues with hostname.

2. On Unity-side application, ROS configuration is incorrect. Ensure that the `ROS IP Adress` under
   `Robotics > ROS Settings` is set to the IP address of the ROS-side computer where this package is running.
