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

import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import SetEnvironmentVariable
import sys

def generate_launch_description():

    args = sys.argv[1:]
    sitl_world = "yosemite";
    if "--sitl_world" in args:
        sitl_world = args[args.index("--sitl_world")+1]

    if(sitl_world=="yosemite"):
        pkg_shared_directory = 'yosemite_valley'
        launch_file = 'drone_yosemite.launch.py'
    elif(sitl_world=="mcmillan"):
        pkg_shared_directory = 'mcmillan_airfield'
        launch_file = 'drone_mcmillan.launch.py'
    elif(sitl_world=="ksql"):
        pkg_shared_directory = 'ksql_airport'
        launch_file = 'drone_ksql.launch.py'
    elif(sitl_world=="baylands"):
        pkg_shared_directory = 'baylands'
        launch_file = 'drone_baylands.launch.py'

    world_dir = get_package_share_directory(pkg_shared_directory)
    included_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
                world_dir + '/launch/' + launch_file))

    return LaunchDescription([
        Node(package='qgroundcontrol', node_executable='qgroundcontrol-start.sh', output='screen'),
        # Node(package='sitl_launcher', node_executable='drone_tf_broadcast', output='screen'),
        included_launch,
    ])
