#!/bin/bash

readonly THIS_FULL_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
readonly GIT_REPO_ROOT="$(dirname "${THIS_FULL_PATH}")"

cd "${GIT_REPO_ROOT}"

sed -e "s/SCRIPT_FULL_PATH/${THIS_FULL_PATH//\//\\/}/g" ./ros.service > /tmp/ros.service
if ! diff /tmp/ros.service /etc/systemd/system/ros.service; then
  sudo mv /tmp/ros.service /etc/systemd/system/ros.service
  sudo systemctl daemon-reload
  sudo systemctl enable ros.service
fi