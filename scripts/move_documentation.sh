#!/bin/bash

DOC_DIR=$(cd "$(dirname "$0")/../catkin_ws/src/rvl_ur_robotiq/documentation/build" >/dev/null 2>&1 ; pwd -P )
SCRIPT_DIR="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"

cp -TRv $(realpath ${DOC_DIR}/html) $(realpath ${SCRIPT_DIR}/../../UR-Robotiq-Integrated-Driver/documentation/html)
cp -TRv $(realpath ${DOC_DIR}/latex/rvl_driver_documentation.pdf) $(realpath ${SCRIPT_DIR}/../../UR-Robotiq-Integrated-Driver/documentation/rvl_driver_documentation.pdf)