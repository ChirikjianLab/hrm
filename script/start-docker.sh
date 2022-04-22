#!/bin/bash

# stop execution instantly. Also print the error location of the running code.
set -e

# Docker image to pull
dockerImage="robotma/focal-dev-qianli:latest"

if [[ "$(docker images -q "$dockerImage" 2> /dev/null)" == "" ]]; then
    docker pull "$dockerImage"
fi

# Name of the docker container
name="highway-roadmap"
user="robot"

# Workspace to mount to docker container
workspace="$HOME"

# TODO : the goal is to be able to use clion with docker for seamless development under a controlled environment
# this hasn't fully working yet but it doesn't affect the launch of the docker container
# X forwarding
if [[ -n ${XAUTHORITY} && -s ${XAUTHORITY} ]]; then
  XAUTHORITY_FILE="-v ${XAUTHORITY}:/home/robot/.Xauthority"
elif [[ -f $HOME/.Xauthority ]]; then
  XAUTHORITY_FILE="-v ${HOME}/.Xauthority:/home/robot/.Xauthority"
fi

docker run \
    -it \
    --rm \
    --ipc host \
    --privileged \
    --name $name \
    --user $user \
    --env="TERM=xterm-256color" \
    -v "$workspace/.ssh/":/home/robot/.ssh/ \
    -v "$workspace/HighwayRoadMap_WS/":/home/robot/HighwayRoadMap_WS/ \
    -v "${HOME}/clion-2022.1:/home/robot/clion-2022.1" \
    -v "${HOME}/.config/JetBrains/CLion2022.1:"/home/robot/.config/JetBrain/CLion2022.1 \
    -v "$workspace/.java/.userPrefs":/home/robot/.java/.userPrefs \
    -v "/tmp/.X11-unix/":/tmp.X11-unix:rw \
    ${XAUTHORITY_FILE} \
    -e HOME=/home/robot \
    -e DISPLAY=$DISPLAY \
    "$dockerImage" \
    /bin/bash
