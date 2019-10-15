#!/usr/bin/env bash

set -x
set -e


# get path to current directory
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"


if [ -z $1 ]
then
  image_name='local_docker_deploy'
else
  image_name=$1
fi

user_id=$(id -u)
image_plus_tag=$image_name:$(date +%Y_%b_%d_%H%M)

docker build --rm -t $image_plus_tag --build-arg user_id=$user_id $DIR
docker tag $image_plus_tag $image_name:latest

echo "Built $image_plus_tag and tagged as $image_name:latest"
