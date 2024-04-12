#!/bin/bash

set -euo nounset

readonly GIT_REPO_ROOT="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"

cd "${GIT_REPO_ROOT}"

git add -A
git stash
git stash drop

readonly GIT_REPO_SHORT_SHA="$(git rev-parse HEAD)"

docker build \
  --tag "ros:${GIT_REPO_SHORT_SHA}" .

docker tag "ros:${GIT_REPO_SHORT_SHA}" ros:latest
