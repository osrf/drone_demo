#!/usr/bin/env bash


if [ -z $1 ]
then
  IMG='local_docker_deploy'
else
  IMG=''
fi

ARGS=("$@")

rocker --x11 --nvidia --user --home --pulse  $IMG ${ARGS[@]}


