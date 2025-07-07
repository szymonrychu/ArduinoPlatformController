#!/bin/bash
set -euo nounset

readonly CURRENT_PWD="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"

sudo systemctl stop ros_robot_platform ros_core
sleep 5

cd ../..
catkin_make "-DCMAKE_INSTALL_PREFIX=/opt/ros/noetic"

sleep 5
sudo systemctl start ros_robot_platform ros_core