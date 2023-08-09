#!/bin/bash

readonly GIT_REPO_ROOT="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"

cd "${GIT_REPO_ROOT}"

readonly GIT_REPO_SHORT_SHA="$(git rev-parse HEAD)"
readonly GIT_CURRENT_BRANCH="$(git rev-parse --abbrev-ref HEAD)"

git fetch origin
git reset "origin/${GIT_CURRENT_BRANCH}"

git reset --hard
git clean -xfd

docker build \
  --tag ros .

docker run \
  --rm \
  --privileged \
  --add-host overlord:192.168.1.10 \
  --network host \
  -e "GIT_COMMIT=${GIT_REPO_SHORT_SHA}" \
  -e "ROS_HOSTNAME=robot" \""
  -v /dev:/dev \
  -v ${HOME}/ros_tmp:/tmp/
  ros:latest roslaunch robot_platform robot_platform.launch


