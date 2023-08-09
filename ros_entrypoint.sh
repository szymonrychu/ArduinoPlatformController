#!/bin/bash

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash" --

set -e
set -o nounset
set -o pipefail

exec "$@"