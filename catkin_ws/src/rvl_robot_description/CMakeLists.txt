cmake_minimum_required(VERSION 3.0.2)
project(rvl_robot_description)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY config launch models xacro
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})