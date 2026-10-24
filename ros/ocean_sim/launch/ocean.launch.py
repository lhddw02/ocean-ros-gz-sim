"""Run Gazebo Harmonic and a one-way ROS bridge with coordinated shutdown."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _start(context):
    share = Path(get_package_share_directory('ocean_sim'))
    gui = LaunchConfiguration('gui').perform(context).strip().lower()
    if gui not in ('true', 'false', '1', '0'):
        raise ValueError('gui must be true or false')
    verbosity = LaunchConfiguration('verbosity').perform(context)
    if verbosity not in ('0', '1', '2', '3', '4'):
        raise ValueError('verbosity must be 0 through 4')
    world = Path(LaunchConfiguration('world').perform(context)).expanduser()
    bridge_config = Path(LaunchConfiguration('bridge_config').perform(context))
    if not world.is_file():
        raise FileNotFoundError('Gazebo world does not exist: ' + str(world))
    if not bridge_config.is_file():
        raise FileNotFoundError('Bridge configuration does not exist: ' + str(bridge_config))

    resources = []
    extra_resources = LaunchConfiguration('extra_resource_path').perform(context)
    if extra_resources:
        resources.append(extra_resources)
    resources.extend([str(share / 'models'), str(share / 'worlds')])
    current_resources = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    if current_resources:
        resources.append(current_resources)

    start_paused = LaunchConfiguration('start_paused').perform(context).lower()
    if start_paused not in ('true', 'false', '1', '0'):
        raise ValueError('start_paused must be true or false')
    command = ['gz', 'sim', '-v', verbosity, '--render-engine', 'ogre2']
    if start_paused in ('false', '0'):
        command.append('-r')
    if gui in ('false', '0'):
        command.append('-s')
    command.append(str(world))
    gazebo = ExecuteProcess(
        cmd=command,
        name='gazebo',
        output='screen',
        sigterm_timeout='10',
        sigkill_timeout='5',
    )
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ocean_bridge',
        output='screen',
        parameters=[{'config_file': str(bridge_config)}],
    )

    return [
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', ':'.join(resources)),
        RegisterEventHandler(OnProcessExit(
            target_action=gazebo,
            on_exit=[EmitEvent(event=Shutdown(reason='Gazebo exited'))],
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=bridge,
            on_exit=[EmitEvent(event=Shutdown(reason='ROS bridge exited'))],
        )),
        gazebo,
        bridge,
    ]


def generate_launch_description():
    share = Path(get_package_share_directory('ocean_sim'))
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='Open the Gazebo GUI (false runs server only)'),
        DeclareLaunchArgument('verbosity', default_value='3',
                              description='Gazebo logging level: 0 to 4'),
        DeclareLaunchArgument('start_paused', default_value='false',
                              description='Wait for a world-control request before advancing physics'),
        DeclareLaunchArgument('extra_resource_path', default_value='',
                              description='Additional model roots, searched before packaged models'),
        DeclareLaunchArgument('world', default_value=str(share / 'worlds' / 'ocean.sdf'),
                              description='Absolute SDF world path'),
        DeclareLaunchArgument('bridge_config', default_value=str(share / 'config' / 'bridge.yaml'),
                              description='Bridge YAML; match its topics to a custom world name'),
        OpaqueFunction(function=_start),
    ])
