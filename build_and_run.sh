#!/bin/bash

readonly GIT_REPO_ROOT="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"

cd "${GIT_REPO_ROOT}"

readonly GIT_REPO_SHORT_SHA="$(git rev-parse HEAD)"
readonly GIT_CURRENT_BRANCH="$(git rev-parse --abbrev-ref HEAD)"


docker build \
  --tag "ros:${GIT_REPO_SHORT_SHA}" .

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
  "ros:${GIT_REPO_SHORT_SHA}" roslaunch robot_platform all.launch


