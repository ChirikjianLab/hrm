#!/bin/bash

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

docker run \
    -it \
    --rm \
    --privileged \
    --name $name \
    --user $user \
    --env="TERM=xterm-256color" \
    -v "$workspace/.ssh/":/home/robot/.ssh/ \
    -v "$workspace/HighwayRoadMap_WS/":/home/robot/HighwayRoadMap_WS/ \
    "$dockerImage" \
    /bin/bash
