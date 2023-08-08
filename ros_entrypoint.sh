#!/bin/bash

sudo /lib/systemd/systemd-udevd --daemon
sudo udevadm control --reload-rules
sudo udevadm trigger

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash" --

set -e
set -o nounset
set -o pipefail

sleep 5

bash -ce "$@"