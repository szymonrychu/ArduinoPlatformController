#!/bin/bash

set -euo nounset

readonly GIT_REPO_ROOT="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"

cd "${GIT_REPO_ROOT}"

git add -A
git stash
git stash drop || true

git pull

readonly GIT_REPO_SHORT_SHA="$(git rev-parse HEAD)"

docker build \
  --tag "arduino-platform-controller:${GIT_REPO_SHORT_SHA}" .

docker tag "arduino-platform-controller:${GIT_REPO_SHORT_SHA}" arduino-platform-controller:latest

./systemd/install_services.sh

sudo systemctl restart ros
