#!/bin/bash

readonly THIS_FULL_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
readonly GIT_REPO_ROOT="$(dirname "${THIS_FULL_PATH}")"

cd "${GIT_REPO_ROOT}"

sed -e "s/SCRIPT_FULL_PATH/${GIT_REPO_ROOT//\//\\/}/g" ./ros.service > /tmp/ros.service

service_updated=""

if ! diff /tmp/ros.service /etc/systemd/system/ros.service; then
  sudo mv /tmp/ros.service /etc/systemd/system/ros.service
  service_updated="${service_updated}Y"
fi

sed -e "s/SCRIPT_FULL_PATH/${GIT_REPO_ROOT//\//\\/}/g" ./code.service > /tmp/code.service
if ! diff /tmp/code.service /etc/systemd/system/code.service; then
  sudo mv /tmp/code.service /etc/systemd/system/code.service
  service_updated="${service_updated}Y"
fi

if [[ -n "${service_updated}" ]]; then
  sudo systemctl daemon-reload
  sudo systemctl enable ros.service
  sudo systemctl enable code.service
fi