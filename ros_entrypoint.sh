#!/bin/bash

sudo /lib/systemd/systemd-udevd --daemon
sudo udevadm control --reload-rules
sudo udevadm trigger

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash" --

set -e
set -o nounset
set -o pipefail

sudo chmod 666 /dev/serial/by-id/*
sudo chown ros /dev/serial/by-id/*

exec "$@"