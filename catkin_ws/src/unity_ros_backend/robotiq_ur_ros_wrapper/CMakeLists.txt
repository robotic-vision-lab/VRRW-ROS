cmake_minimum_required(VERSION 3.0.2)
project(robotiq_ur_ros_wrapper)

find_package(catkin REQUIRED COMPONENTS rospy roslaunch std_msgs sensor_msgs message_generation)

add_message_files(DIRECTORY msg)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package()

catkin_install_python(PROGRAMS 
    src/robotiq_ros_node/RobotiqRosNode.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})