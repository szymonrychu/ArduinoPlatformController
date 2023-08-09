#!/bin/bash

readonly THIS_FULL_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
readonly GIT_REPO_ROOT="$(dirname "${THIS_FULL_PATH}")"

cd "${GIT_REPO_ROOT}"

readonly GIT_CURRENT_BRANCH="$(git rev-parse --abbrev-ref HEAD)"

git fetch origin
git reset "origin/${GIT_CURRENT_BRANCH}"

git reset --hard
git clean -xfd

sudo sed "s/SCRIPT_FULL_PATH/${THIS_FULL_PATH}/g" ${GIT_REPO_ROOT}/ros.service > /etc/systemd/system/ros.service
sudo systemctl daemon-reload

docker build -t ros .

docker run \
  --rm \
  --privileged \
  --add-host overlord:192.168.1.10 \
  --network host \
  -e "ROS_HOSTNAME=robot" \
  -e "ROS_MASTER_URI=http://robot:11311/" \
  -v /dev:/dev \
  -it ros:latest \
    roslaunch robot_platform robot_platform.launch


