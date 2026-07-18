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
#
# Derived from nav2_bringup's tb3_simulation_launch.py and nav2_minimal_tb3_sim
# (Open Source Robotics Foundation / Open Navigation LLC, Apache-2.0).

"""
Launch a minimal TurtleBot3 Gazebo simulation for the emcl2 demo.

This brings up only the simulator (Gazebo world, robot, ros_gz bridge and
robot_state_publisher) using ``nav2_minimal_tb3_sim``. Localization (emcl2)
and navigation are launched separately so that the demo shows emcl2 running
in place of nav2's amcl.
"""

import os
import re
import tempfile

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

from launch_ros.actions import Node


def generate_launch_description():
    sim_dir = get_package_share_directory('nav2_minimal_tb3_sim')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_simulator = LaunchConfiguration('use_simulator')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')
    robot_name = LaunchConfiguration('robot_name')
    robot_sdf = LaunchConfiguration('robot_sdf')
    pose = {
        'x': LaunchConfiguration('x_pose', default='-2.00'),
        'y': LaunchConfiguration('y_pose', default='-0.50'),
        'z': LaunchConfiguration('z_pose', default='0.01'),
        'R': LaunchConfiguration('roll', default='0.00'),
        'P': LaunchConfiguration('pitch', default='0.00'),
        'Y': LaunchConfiguration('yaw', default='0.00'),
    }

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true',
    )

    declare_use_simulator_cmd = DeclareLaunchArgument(
        'use_simulator',
        default_value='True',
        description='Whether to start the simulator',
    )

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher',
    )

    declare_headless_cmd = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Whether to run Gazebo headless (without the GUI client)',
    )

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(sim_dir, 'worlds', 'tb3_sandbox.sdf.xacro'),
        description='Full path to world model file to load',
    )

    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name', default_value='turtlebot3_waffle', description='name of the robot'
    )

    declare_robot_sdf_cmd = DeclareLaunchArgument(
        'robot_sdf',
        default_value=os.path.join(sim_dir, 'urdf', 'gz_waffle.sdf.xacro'),
        description='Full path to robot sdf file to spawn the robot in gazebo',
    )

    urdf = os.path.join(sim_dir, 'urdf', 'turtlebot3_waffle.urdf')
    with open(urdf, 'r') as infp:
        robot_description = infp.read()

    # turtlebot3_waffle.urdf points its meshes at models/<name>.dae, but the
    # meshes actually live under models/turtlebot3_model/meshes/. Gazebo uses
    # the (correct) SDF, so only RViz's RobotModel display hits this and reports
    # "Errors loading geometries". Rewrite the mesh paths to the real location.
    # The regex only matches meshes directly under models/, so it becomes a
    # no-op once the upstream URDF is fixed.
    robot_description = re.sub(
        r'(package://nav2_minimal_tb3_sim/models/)([^/]+\.dae)',
        r'\1turtlebot3_model/meshes/\2',
        robot_description,
    )

    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time, 'robot_description': robot_description}
        ],
        remappings=remappings,
    )

    # The world is a xacro file so that the SceneBroadcaster plugin can be
    # toggled by the headless flag. Gazebo cannot take an SDF string for the
    # world, so xacro output is written to a temporary file first.
    world_sdf = tempfile.mktemp(prefix='emcl2_', suffix='.sdf')
    world_sdf_xacro = ExecuteProcess(
        cmd=['xacro', '-o', world_sdf, ['headless:=', headless], world]
    )
    remove_temp_sdf_file = RegisterEventHandler(
        event_handler=OnShutdown(
            on_shutdown=[OpaqueFunction(function=lambda _: os.remove(world_sdf))]
        )
    )

    gazebo_server = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-s', world_sdf],
        output='screen',
        condition=IfCondition(use_simulator),
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
            )
        ),
        condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])),
        launch_arguments={'gz_args': ['-v4 -g ']}.items(),
    )

    # spawn_tb3.launch.py spawns the robot, starts the ros_gz bridge and sets
    # the GZ_SIM_RESOURCE_PATH environment variables.
    spawn_robot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_dir, 'launch', 'spawn_tb3.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'robot_name': robot_name,
            'robot_sdf': robot_sdf,
            'x_pose': pose['x'],
            'y_pose': pose['y'],
            'z_pose': pose['z'],
            'roll': pose['R'],
            'pitch': pose['P'],
            'yaw': pose['Y'],
        }.items(),
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_headless_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_robot_sdf_cmd)

    ld.add_action(world_sdf_xacro)
    ld.add_action(remove_temp_sdf_file)
    # spawn_tb3.launch.py appends the model directories to GZ_SIM_RESOURCE_PATH,
    # so it must run before the Gazebo server starts or the world's model://
    # includes cannot be resolved.
    ld.add_action(spawn_robot_cmd)
    ld.add_action(gazebo_server)
    ld.add_action(gazebo_client)
    ld.add_action(start_robot_state_publisher_cmd)

    return ld
