# docker_deploy branch

This branch has the contents of the Dockerfile used to deploy the drone demo.

## To build a local copy run

`./build.bash`

If you'd like to give it a non-default name you can pass a second argument.
The default image name is `local_docker_deploy`

## To run invoke

`./run.bash`

The default argument is `local_docker_deploy`.

If you want to run the prebuilt image use `./run.bash tfoote/drone_demo`

