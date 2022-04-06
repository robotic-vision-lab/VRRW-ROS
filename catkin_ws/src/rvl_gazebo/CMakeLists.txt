cmake_minimum_required(VERSION 3.0.2)
project(rvl_gazebo)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    gazebo_ros
    image_transport
    camera_info_manager
    sensor_msgs
)

find_package(gazebo REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

catkin_package(
    DEPENDS
        roscpp
        gazebo_ros
        image_transport
        camera_info_manager
        sensor_msgs
)

include_directories(include)

include_directories(
    SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

# individual libraries build target and linking

add_library(
    realsense_l515_gazebo_plugin
    src/RealSensePlugin.cpp
    src/gazebo_ros_realsense.cpp
)

target_link_libraries(realsense_l515_gazebo_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(realsense_l515_gazebo_plugin ${catkin_EXPORTED_TARGETS})

install(
    TARGETS
        realsense_l515_gazebo_plugin
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

foreach(dir include)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

# other dirs

install(DIRECTORY config launch xacro
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})