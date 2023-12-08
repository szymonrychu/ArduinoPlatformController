#!/bin/bash

mkdir -p "${HOME}/config"

docker run --rm -p 8080:8080 \
  -v "$HOME/.local:/home/coder/.local" \
  -v "$HOME/.config:/home/coder/.config" \
  -v "$HOME/.ssh:/home/coder/.ssh" \
  -v "$HOME/.git/coder/.git" \
  -v "/opt/ros/noetic" \
  -v "/:/root" \
  -v "$HOME/ArduinoPlatformController:/home/coder/ArduinoPlatformController" \
  -u "$(id -u):$(id -g)" \
  -e "DOCKER_USER=$USER" \
  codercom/code-server:latest