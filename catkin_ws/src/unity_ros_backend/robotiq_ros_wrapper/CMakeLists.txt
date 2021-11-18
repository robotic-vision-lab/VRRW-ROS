cmake_minimum_required(VERSION 3.0.2)
project(robotiq_ros_wrapper)

find_package(catkin REQUIRED COMPONENTS rospy roslaunch std_msgs sensor_msgs message_generation)

add_message_files(DIRECTORY msg)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

file(GLOB nodes_DIR "nodes/*.py")


catkin_package()

catkin_install_python(PROGRAMS 
    ${nodes_DIR}
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})