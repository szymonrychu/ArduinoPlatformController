#!/bin/bash

ROS_ROOT="${CONDA_PREFIX:-/opt/ros/noetic}"

readonly CATKIN_WS_DIR="$(dirname "$(dirname "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")")")"
cd "/home/deck/catkin_ws"

source "${CONDA_PREFIX}/setup.bash"
catkin_make "-DCMAKE_INSTALL_PREFIX=${CONDA_PREFIX}" install
