cmake_minimum_required(VERSION 3.0.2)
project(rvl_gazebo)

find_package(catkin REQUIRED)

catkin_package()

file(GLOB python_scripts "scripts/*.py")
catkin_install_python(PROGRAMS
  ${python_scripts}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch models xacro
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})