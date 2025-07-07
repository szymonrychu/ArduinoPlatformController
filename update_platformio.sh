#!/bin/bash
set -euo nounset

readonly CURRENT_PWD="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"

sudo systemctl stop ros_robot_platform ros_core
sleep 5

platformio run -e small_robot -t upload

sleep 5
sudo systemctl start ros_robot_platform ros_core