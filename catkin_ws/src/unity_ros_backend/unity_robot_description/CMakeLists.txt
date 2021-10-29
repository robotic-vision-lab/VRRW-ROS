cmake_minimum_required(VERSION 3.0.2)
project(unity_robot_description)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY xacro urdf configs models launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})