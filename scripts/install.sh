#!/bin/bash
source /opt/ros/${ROS_DISTRO}/setup.bash
cd ~/catkin_ws/
catkin_make -DCMAKE_INSTALL_PREFIX=/opt/ros/${ROS_DISTRO} install
