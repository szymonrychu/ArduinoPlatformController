#!/bin/bash

set -euo nounset


sudo systemctl stop ros

readonly GIT_REPO_ROOT="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"

cd "${GIT_REPO_ROOT}"

git fetch origin
git reset origin/master

git add -A
git stash
git stash drop || true

readonly GIT_REPO_SHORT_SHA="$(git rev-parse HEAD)"

docker build \
  --tag "arduino-platform-controller:${GIT_REPO_SHORT_SHA}" .

docker tag "arduino-platform-controller:${GIT_REPO_SHORT_SHA}" arduino-platform-controller:latest

./systemd/install_services.sh

sudo systemctl start ros
