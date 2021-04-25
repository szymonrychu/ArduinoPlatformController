#!/bin/bash
source /opt/ros/${ROS_DISTRO}/setup.bash
cd ~/catkin_ws/
cd src/ArduinoPlatformController/
git stash
git stash clear
git pull
cd ../../
catkin_make -DCMAKE_INSTALL_PREFIX=/opt/ros/${ROS_DISTRO} install
