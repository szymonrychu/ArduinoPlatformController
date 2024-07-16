#!/bin/bash

set -euo nounset

readonly ROS_PACKAGE="${1}"
readonly ROS_LAUNCH_FILE="${2}"
readonly GIT_REPO_ROOT="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"

cd "${GIT_REPO_ROOT}"

rm -rf "/tmp/${ROS_PACKAGE}-${ROS_LAUNCH_FILE}"
mkdir -p "/tmp/${ROS_PACKAGE}-${ROS_LAUNCH_FILE}"
chown -R 1000:1000 "/tmp/${ROS_PACKAGE}-${ROS_LAUNCH_FILE}"

sudo touch /var/run/shutdown_signal
sudo chmod 666 /var/run/shutdown_signal

docker run \
  --rm \
  --privileged \
  --name "${ROS_PACKAGE}-${ROS_LAUNCH_FILE}" \
  --add-host mobile-overlord:192.168.1.49 \
  --add-host overlord:192.168.1.50 \
  --network host \
  -e "ROS_HOSTNAME=robot" \
  -e "ROS_LOG_DIR=/tmp/ros" \
  -v /dev:/dev \
  -v "/tmp/${ROS_PACKAGE}-${ROS_LAUNCH_FILE}:/tmp/ros" \
  -v /var/run/shutdown_signal:/shutdown_signal \
  "arduino-platform-controller:latest" roslaunch "${ROS_PACKAGE}" "${ROS_LAUNCH_FILE}"
