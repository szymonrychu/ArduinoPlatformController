#!/bin/bash

set -euo nounset

readonly ROS_PACKAGE="${1}"
readonly ROS_LAUNCH_FILE="${2}"
readonly GIT_REPO_ROOT="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"

cd "${GIT_REPO_ROOT}"

rm -rf /tmp/ros
mkdir -p /tmp/ros
chown -R 1000:1000 /tmp/ros

docker run \
  --rm \
  --privileged \
  --name ros \
  --add-host mobile-overlord:192.168.1.49 \
  --add-host overlord:192.168.1.50 \
  --network host \
  -e "ROS_HOSTNAME=robot" \
  -v /dev:/dev \
  -v /tmp/ros:/home/ros/.ros \
  "arduino-platform-controller:latest" roslaunch "${ROS_PACKAGE}" "${ROS_LAUNCH_FILE}"
