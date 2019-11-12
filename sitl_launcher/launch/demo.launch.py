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
from launch import LaunchService
from launch_ros import get_default_launch_description
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch import LaunchContext
from launch.utilities import perform_substitutions, normalize_to_list_of_substitutions

from launch.actions import IncludeLaunchDescription
from launch import LaunchDescriptionSource

def generate_launch_description():

    pkg_shared_directory = "yosemite_valley"
    launch_file = 'drone_yosemite.launch.py'

    world_dir = get_package_share_directory(pkg_shared_directory)
    included_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
                world_dir + '/launch/' + launch_file))

    use_rviz = LaunchConfiguration('use_rviz')
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='False',
        description='Whether to start RVIZ')

    use_qgroundcontrol = LaunchConfiguration('use_qgroundcontrol')
    declare_use_qgroundcontrol_cmd = DeclareLaunchArgument(
        'use_qgroundcontrol',
        default_value='False',
        description='Whether to start QGroundcontrol')

    include_nodes = [included_launch]

    sitl_launcher_dir = get_package_share_directory('sitl_launcher')

    start_rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        node_executable='rviz2',
        node_name='rviz2',
        arguments=['-d', sitl_launcher_dir + "/rviz/rviz.rviz"],
        output='screen')

    start_qgroundcontrol_cmd = Node(
        condition=IfCondition(use_qgroundcontrol),
        package='qgroundcontrol',
        node_executable='qgroundcontrol-start.sh',
        node_name='qgroundcontrol',
        output='screen')

    ld = LaunchDescription(include_nodes)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(start_rviz_cmd)
    ld.add_action(declare_use_qgroundcontrol_cmd)
    ld.add_action(start_qgroundcontrol_cmd)

    return ld
