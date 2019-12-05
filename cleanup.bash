#!/usr/bin/bash

docker images | grep local_docker_deploy
docker images | grep local_docker_deploy | awk '{print $1 ":" $2}' | xargs -L 1 docker rmi
