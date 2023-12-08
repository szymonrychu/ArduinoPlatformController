#!/bin/bash

mkdir -p "${HOME}/config"

docker run --rm -p 8080:8080 \
  -v "$HOME/.local:/home/coder/.local" \
  -v "$HOME/.config:/home/coder/.config" \
  -v "$HOME/.ssh:/home/coder/.ssh" \
  -v "$HOME/.gitconfig:/home/coder/.gitconfig" \
  -v "$HOME/.bash_history:/home/coder/.bash_history" \
  -v "$HOME/ArduinoPlatformController:/home/coder/ArduinoPlatformController" \
  -u "$(id -u):$(id -g)" \
  --add-host robot:192.168.1.48 \
  -e "DOCKER_USER=$USER" \
  codercom/code-server:latest
