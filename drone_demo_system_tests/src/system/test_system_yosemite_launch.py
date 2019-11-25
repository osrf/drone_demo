#!/usr/bin/env python3
# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import os
import sys

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch import LaunchService
from launch.actions import ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_testing.legacy import LaunchTestService


def generate_launch_description():

    os.environ["PX4_HOME_LAT"] = "37.7332531"
    os.environ["PX4_HOME_LON"] = "-119.5616378"
    os.environ["PX4_HOME_ALT"] = "2800.4"

    gazebo_dir = get_package_share_directory('gazebo_ros')
    included_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_dir + '/launch/gazebo.launch.py'),
        launch_arguments={'world': 'worlds/yosemite.world',
                          'paused': 'false',
                          'physics': 'ode',
                          'gui': 'false'}.items()
        )

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1'),
        included_launch,
        Node(package='sitl_launcher', node_executable='launch_drone_ros2.py',
             output='screen', arguments=['--iris', '0']),
    ])


def main(argv=sys.argv[1:]):
    ld = generate_launch_description()

    waypoints_dir = get_package_share_directory('drone_demo_system_tests')

    test1_action = ExecuteProcess(
        cmd=[os.path.join(os.getenv('TEST_DIR'), 'tester_node.py'),
             '-f', waypoints_dir + '/waypoints/waypoints.txt',
             '--ros-args', '--remap', '__ns:=/iris_0'],
        name='tester_node',
        output='screen')

    lts = LaunchTestService()
    lts.add_test_action(ld, test1_action)
    ls = LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return lts.run(ls)


if __name__ == '__main__':
    sys.exit(main())
