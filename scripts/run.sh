#!/bin/bash

while ! curl google.com > /dev/null >&1; do
    echo "Waiting for network!"
    sleep 1
done

source /opt/ros/noetic/setup.bash
ROS_HOSTNAME=robot
ROS_MASTER_URI=http://robot:11311
ROS_DISTRO=noetic

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
"${SCRIPT_DIR}/refresh.sh"
"${SCRIPT_DIR}/install.sh"

roslaunch robot_platform robot_platform.launch