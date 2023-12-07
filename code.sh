
#!/bin/bash

mkdir -p "${HOME}/config"

docker run --rm -it --name code-server -p 8080:8080 \
  -v "$HOME/.local:/home/coder/.local" \
  -v "$HOME/.config:/home/coder/.config" \
  -v "$HOME/.ssh:/home/coder/.ssh" \
  -v "/opt/ros/noetic:/opt/ros/noetic" \
  -v "$HOME/ArduinoPlatformController:/home/coder/ArduinoPlatformController" \
  -u "$(id -u):$(id -g)" \
  -e "DOCKER_USER=$USER" \
  codercom/code-server:latest