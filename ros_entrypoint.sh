#!/bin/bash

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash" --

set -e
set -o nounset
set -o pipefail

sudo chmod 666 /dev/serial/by-id/*

exec "$@"