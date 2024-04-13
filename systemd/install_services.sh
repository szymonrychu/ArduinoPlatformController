#!/bin/bash

readonly THIS_FULL_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
readonly THIS_ROOT="$(dirname "${THIS_FULL_PATH}")"

cd "${THIS_ROOT}"

sed -e "s/SCRIPT_FULL_PATH/${THIS_ROOT//\//\\/}/g" ./ros.service > /tmp/ros.service

service_updated=""

if ! diff /tmp/ros.service /etc/systemd/system/ros.service; then
  sudo mv /tmp/ros.service /etc/systemd/system/ros.service
  service_updated="${service_updated}Y"
fi

if [[ -n "${service_updated}" ]]; then
  sudo systemctl daemon-reload
  sudo systemctl enable ros.service
fi