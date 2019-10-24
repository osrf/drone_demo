This repository contains the contents for a drone demonstration.

It is running Gazebo, PX4, QGroundControl and several other components.

You will need `rocker` `docker` and `nvidia-docker` installed. See the prerequisite instructions here:

https://github.com/osrf/px4sitl/blob/master/install.md


There are also some notes about how to make it work without an nvidia card as well. Mileage may vary.

To run the demo from the hosted docker images you can simply invoke

`rocker --x11 --nvidia --user --home --pulse tfoote/drone_demo`

## Modifying or building your own 

See the Dockerfiles and docs in the `docker_deploy` branch: https://github.com/osrf/drone_demo/tree/docker_deploy

## What you'll see when you launch

By default it will launch gazebo headless, QGroundControl, and a small python script that will prompt you for how many of what type of drones to launch. 

You can select up to 4 total drones.

After that gazebo models of the instances will be spawned with appropriately parameterized PX4 autopilots connected.
Once they have initialized you can interact with them via QGroundControl.


### Changing Behavior

If you'd like to change behavior you can pass argumets with a command like this:

`rocker --x11 --nvidia --user --home --pulse tfoote/drone_demo roslaunch sitl_launcher demo.launch gui:=false sitl_world:=yosemite`

`gui:=true` will enable the Gazebo head

Other valid sitl_worlds are: `mcmillan`, `ksql`, and `baylands`.

You can also pass arguments via `drone_args` to skip the drone selector gui and immediately start the selected drones. 

The script is here: https://github.com/osrf/drone_demo/blob/master/sitl_launcher/scripts/launch_drone