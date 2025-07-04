#!/bin/bash

ROS_ROOT="${CONDA_PREFIX:-/opt/ros/noetic}"

readonly CATKIN_WS_DIR="$(dirname "$(dirname "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")")")"
cd "${CATKIN_WS_DIR}"

source "${CONDA_PREFIX}/setup.bash"

catkin_make "-DCMAKE_INSTALL_PREFIX=${CONDA_PREFIX}" install
catkin_make "-DCMAKE_INSTALL_PREFIX=/opt/ros/noetic" install