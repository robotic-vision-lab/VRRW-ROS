cmake_minimum_required(VERSION 3.0.2)
project(ur_remote_dashboard)

find_package(catkin REQUIRED COMPONENTS rospy roslaunch std_msgs sensor_msgs geometry_msgs moveit_msgs trajectory_msgs)

catkin_python_setup()

catkin_package()

file(GLOB python_scripts_DIR "scripts/*.py")

catkin_install_python(PROGRAMS ${python_scripts_DIR} DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})