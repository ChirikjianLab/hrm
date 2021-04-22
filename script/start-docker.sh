#!/bin/bash

# Docker image to pull
dockerImage="robotma/focal-dev-qianli:latest"

if [[ "$(docker images -q "$dockerImage" 2> /dev/null)" == "" ]]; then
    docker pull "$dockerImage"
fi

# Name of the docker container
name="highway-roadmap"

# Workspace to mount to docker container
workspace="$HOME/HighwayRoadMap_WS"

docker run \
    -it \
    --rm \
    --name $name \
    --env="TERM=xterm-256color" \
    -v $workspace/HighwayRoadMap:$HOME/HighwayRoadMap \
    "$dockerImage" \
    /bin/bash
