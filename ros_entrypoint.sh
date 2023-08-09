#!/bin/bash

readonly GIT_COMMIT="${GIT_COMMIT:-none}"
touch /tmp/git_sha

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash" --

set -e
set -o nounset
set -o pipefail

sudo chmod 666 /dev/serial/by-id/*

if [[ "${GIT_COMMIT}" == "none" ]] || [[ "$(cat /tmp/git_sha)" != "${GIT_COMMIT}" ]]; then
  cd /home/ros/arduino
  platformio run -t upload -e small_robot
  cd -
  echo "${GIT_COMMIT}" > /tmp/git_sha
fi

exec "$@"