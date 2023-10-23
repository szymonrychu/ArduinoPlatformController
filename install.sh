#!/bin/bash

readonly CATKIN_WS_DIR="$(dirname "$(dirname "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")")")"
cd "${CATKIN_WS_DIR}"

source /opt/ros/noetic/setup.bash

catkin_make -DCMAKE_INSTALL_PREFIX=/opt/ros/noetic install