#!/bin/bash

readonly GIT_REPO_ROOT="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"

cd "${GIT_REPO_ROOT}"

readonly GIT_CURRENT_BRANCH="$(git rev-parse --abbrev-ref HEAD)"

git fetch origin
git reset "origin/${GIT_CURRENT_BRANCH}"

git reset --hard
git clean -xfd

docker build -t ros .

docker run \
  --rm \
  --privileged \
  --add-host overlord:192.168.1.10 \
  --network host \
  -e "ROS_HOSTNAME=robot" \
  -e "ROS_MASTER_URI=http://robot:11311/" \
  -v /dev:/dev \
  ros:latest roslaunch robot_platform robot_platform.launch


