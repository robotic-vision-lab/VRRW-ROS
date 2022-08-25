# if .bash_aliases exists, it WILL be source by default bash (at least on Ubuntu)
# so you can actually define some configs here...

# ------------------------------- BASH CONFIGS ------------------------------- #

# always source the default catkin setup.bash
source ${COLCON_WS}/install/setup.bash

# auto cd into catkin workspace instead of / (base path)
cd ${COLCON_WS}

# nice, shorter prompt
PS1='\n\[\e[0;38;5;122m\]\u \[\e[0;38;5;147m\]\w\n\[\e[0m\]> \[\e[0m\]'

# -------------------------------- ROS ALIASES ------------------------------- #

alias src_ros='source ${COLCON_WS}/install/setup.bash'

alias run_rosdep='cd ${COLCON_WS} && rosdep update && rosdep install --from-paths src --ignore-src -r -y'

alias build_colcon='cd ${COLCON_WS} && colcon build --symlink-install --cmake-args -Wno-dev -DCMAKE_BUILD_TYPE=Debug && src_ros'

alias rebuild_colcon='rm -rf ${CONCOL_WS}/install ${CONCOL_WS}/build ${CONCOL_WS}/log && build_colcon && src_ros'

# alias start_fake_robot='ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=yyy.yyy.yyy.yyy initial_joint_controller:=joint_trajectory_controller use_fake_hardware:=true launch_rviz:=false'

# alias start_fake_moveit='ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true use_fake_hardware:=true'

echo "predefined aliases: src_ros, run_rosdep, build_colcon, rebuild_colcon"