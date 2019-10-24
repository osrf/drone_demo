#!/bin/sh

. /usr/share/gazebo/setup.sh
. /workspace/drone_demo_ros2/install/setup.sh
rm -r ~/.gazebo/paging/

exec "$@"
