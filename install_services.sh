#!/bin/bash

readonly THIS_FULL_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
readonly GIT_REPO_ROOT="$(dirname "${THIS_FULL_PATH}")"

cd "${GIT_REPO_ROOT}"

sed -e "s/SCRIPT_FULL_PATH/${GIT_REPO_ROOT//\//\\/}/g" ./wheel_controller.service > /tmp/wheel_controller.service
sed -e "s/SCRIPT_FULL_PATH/${GIT_REPO_ROOT//\//\\/}/g" ./joy_controller.service > /tmp/joy_controller.service
sed -e "s/SCRIPT_FULL_PATH/${GIT_REPO_ROOT//\//\\/}/g" ./rplidar.service > /tmp/rplidar.service

service_updated=""

if ! diff /tmp/wheel_controller.service /etc/systemd/system/wheel_controller.service; then
  sudo mv /tmp/wheel_controller.service /etc/systemd/system/wheel_controller.service
  service_updated="${service_updated}Y"
fi

if ! diff /tmp/joy_controller.service /etc/systemd/system/joy_controller.service; then
  sudo mv /tmp/joy_controller.service /etc/systemd/system/joy_controller.service
  service_updated="${service_updated}Y"
fi

if ! diff /tmp/rplidar.service /etc/systemd/system/rplidar.service; then
  sudo mv /tmp/rplidar.service /etc/systemd/system/rplidar.service
  service_updated="${service_updated}Y"
fi

if [[ -n "${service_updated}" ]]; then
  sudo systemctl daemon-reload
  sudo systemctl enable wheel_controller.service
  sudo systemctl enable joy_controller.service
  sudo systemctl enable rplidar.service
fi