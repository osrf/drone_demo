# Development Docker Containers

This repo contains Dockerfiles that are handy for ros and gazebo development.
It requires `nvidia-docker2`.
colcon and vcstool are recommended too.

## Docker Images requireing nvidia-docker2

## Quick-start

```
# Build the base image to create `developer` user with your user id
./build.bash devbase-xenial
# Build image with gazebo9 binary installation
./build.bash gz9
# Create a temporary container from the image
./run.bash gz9
# Now inside docker container
gazebo --verbose
```

## Scripts

### ./build.bash
Wrapper for `docker build` that passes the current user id as a build argument.

### ./run.bash
Wrapper for `docker run` that sets up useful stuff:
* spacenav
* X server for GUI
* nvidia-docker2 runtime
* mounting volumes
* vimrc
* settings for running gdb

`./run.bash image-name some/file/path some/other/file/path ...`


## Devbase images
* devbase-bionic
* devbase-xenial
* devbase-trusty

These are based off of nvidia images with beta support for opengl using nvidia-docker2.
Each provides a different ubuntu version.
All have a user named `developer` with NOPASSWD sudo access and a user id matching the user outside the container.
You must build these images on your local machine using `./build.bash devbase-bionic` etc.
The images also have other tools I find useful (colcon, python3-venv, cmake, gdb, ...).

## Gazebo from source images
* gz-dev
* gz7-dev
* gz7-trusty-dev

These images come with dependencies needed to build sdformat, ignition, and gazebo libraries.
The expectation is that the full stack (sdformat, ign-..., gazebo) will be built from source.
I use a combination of vcstool and colcon to do this.

## Gazebo binary images
* gz7
* gz7-trusty
* gz8
* gz9

These images contain gazebo as installed from `packages.osrfoundation.org`.

## ROS2 from source images
* ros2-bionic

Image for building ros2 completely from source.

## ROS 1 binary images
* ros-melodic-gazebo9
  * ROS Melodic (desktop full) on Ubuntu Bionic (18.04) with Gazebo 9 from `packages.osrfoundation.org`

## Other images

There are a lot of other images in this repo.
Most are not usable with because they have not been ported from nvidia-docker1 yet.

---
*Inspired by [scpeters](https://bitbucket.org/scpeters/unix-stuff) gazebo8-docker*
