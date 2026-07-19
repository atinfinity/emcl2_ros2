# Copyright 2026 emcl2_ros2 developers
# SPDX-FileCopyrightText: 2026 emcl2_ros2 developers
# SPDX-License-Identifier: LGPL-3.0-or-later
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

"""
Headless simulation bringup for the localization CI job.

Brings up a headless Gazebo (tb3_sandbox world), spawns the TurtleBot3 with a
ground-truth OdometryPublisher, bridges the sim topics plus /ground_truth,
runs robot_state_publisher and emcl2. No GUI and no RViz. The world_sdf and
robot_sdf are pre-generated (xacro + ground-truth plugin injection) by
run_localization_eval.sh so this file stays free of shell/xacro logic.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    sim_dir = get_package_share_directory('nav2_minimal_tb3_sim')
    emcl2_dir = get_package_share_directory('emcl2')
    nav2_dir = get_package_share_directory('nav2_bringup')

    world_sdf = LaunchConfiguration('world_sdf')
    robot_sdf = LaunchConfiguration('robot_sdf')
    headless_rendering = LaunchConfiguration('headless_rendering')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    robot_x = LaunchConfiguration('robot_x')
    robot_y = LaunchConfiguration('robot_y')
    robot_yaw = LaunchConfiguration('robot_yaw')

    declare_world = DeclareLaunchArgument(
        'world_sdf', description='Full path to the (already xacro-expanded) world SDF'
    )
    declare_robot = DeclareLaunchArgument(
        'robot_sdf',
        description='Full path to the robot SDF (gz_waffle + ground-truth OdometryPublisher)',
    )
    declare_headless_rendering = DeclareLaunchArgument(
        'headless_rendering',
        default_value='false',
        description='Use gz EGL headless rendering (for GPU-less CI runners)',
    )
    declare_map = DeclareLaunchArgument(
        'map', default_value=os.path.join(nav2_dir, 'maps', 'tb3_sandbox.yaml')
    )
    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(emcl2_dir, 'config', 'emcl2_quick_start.param.yaml'),
    )
    declare_robot_x = DeclareLaunchArgument('robot_x', default_value='-2.0')
    declare_robot_y = DeclareLaunchArgument('robot_y', default_value='-0.5')
    declare_robot_yaw = DeclareLaunchArgument('robot_yaw', default_value='0.0')

    # world model:// includes (turtlebot3_world) resolve via GZ_SIM_RESOURCE_PATH.
    set_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(sim_dir, 'models')
    )
    set_resources_parent = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', str(os.path.dirname(sim_dir))
    )

    gz_server = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-s', '-v1', world_sdf],
        output='screen',
        condition=UnlessCondition(headless_rendering),
    )
    gz_server_headless = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-s', '-v1', '--headless-rendering', world_sdf],
        output='screen',
        condition=IfCondition(headless_rendering),
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-world', 'default', '-name', 'turtlebot3_waffle',
            '-x', robot_x, '-y', robot_y, '-z', '0.01', '-Y', robot_yaw,
            '-file', robot_sdf,
        ],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{
            'config_file': os.path.join(
                sim_dir, 'configs', 'turtlebot3_waffle_bridge.yaml'),
            'expand_gz_topic_names': True,
            'use_sim_time': True,
        }],
    )

    gt_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=['/ground_truth@nav_msgs/msg/Odometry[gz.msgs.Odometry'],
        parameters=[{'use_sim_time': True}],
    )

    urdf = os.path.join(sim_dir, 'urdf', 'turtlebot3_waffle.urdf')
    with open(urdf, 'r') as infp:
        robot_description = infp.read()
    import re
    robot_description = re.sub(
        r'(package://nav2_minimal_tb3_sim/models/)([^/]+\.dae)',
        r'\1turtlebot3_model/meshes/\2', robot_description)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True, 'robot_description': robot_description}],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
    )

    emcl2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(emcl2_dir, 'launch', 'emcl2.launch.py')),
        launch_arguments={
            'params_file': params_file,
            'map': map_yaml,
            'use_sim_time': 'true',
        }.items(),
    )

    ld = LaunchDescription()
    ld.add_action(declare_world)
    ld.add_action(declare_robot)
    ld.add_action(declare_headless_rendering)
    ld.add_action(declare_map)
    ld.add_action(declare_params)
    ld.add_action(declare_robot_x)
    ld.add_action(declare_robot_y)
    ld.add_action(declare_robot_yaw)
    ld.add_action(set_resources)
    ld.add_action(set_resources_parent)
    ld.add_action(spawn_robot)
    ld.add_action(gz_server)
    ld.add_action(gz_server_headless)
    ld.add_action(bridge)
    ld.add_action(gt_bridge)
    ld.add_action(robot_state_publisher)
    ld.add_action(emcl2)
    return ld
