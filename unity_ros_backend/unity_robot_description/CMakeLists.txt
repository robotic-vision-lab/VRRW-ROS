cmake_minimum_required(VERSION 3.0.2)
project(unity_robot_description)

find_package(catkin REQUIRED)

catkin_package()

# catkin_install_python(PROGRAMS
#   make_robot.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

install(DIRECTORY xacro urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})