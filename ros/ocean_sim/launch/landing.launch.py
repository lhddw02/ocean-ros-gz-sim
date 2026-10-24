"""Generate a matching AprilTag, start physics paused, then run visual landing."""
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ocean_sim.scene_builder import build_scene, load_configuration


def start(context):
    share = Path(get_package_share_directory('ocean_sim'))
    config = load_configuration(share, LaunchConfiguration('config').perform(context))
    scene = config['scene'].copy()
    for key, converter in [('tag_family', str), ('tag_id', int), ('tag_size', float)]:
        value = LaunchConfiguration(key).perform(context)
        if value and value != 'config':
            scene[key] = converter(value)
    enabled = LaunchConfiguration('autoland').perform(context).lower()
    if enabled not in ('true', 'false', '1', '0'):
        raise ValueError('autoland must be true or false')
    runtime = Path(tempfile.mkdtemp(prefix='ocean-landing-'))
    world, resources = build_scene(share, runtime, scene)
    parameters = dict(config['controller'])
    parameters.update({'tag_family': scene['tag_family'], 'tag_id': int(scene['tag_id']),
                       'tag_size': float(scene['tag_size']), 'enabled': enabled in ('true', '1'),
                       'use_sim_time': True})
    vision = Node(package='ocean_sim', executable='vision_landing',
                  output='screen', parameters=[parameters])
    bootstrap = Node(package='ocean_sim', executable='landing_bootstrap', output='screen',
                     parameters=[{'use_sim_time': True}])
    return [
        LogInfo(msg=f'Landing assets: {runtime}; tag {scene["tag_family"]} ID {scene["tag_id"]}'),
        RegisterEventHandler(OnProcessExit(target_action=vision,
            on_exit=[EmitEvent(event=Shutdown(reason='Landing controller exited'))])),
        RegisterEventHandler(OnProcessExit(target_action=bootstrap,
            on_exit=[EmitEvent(event=Shutdown(reason='Landing startup controller exited'))])),
        vision,
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='world_control_bridge',
             arguments=['/world/ocean/control@ros_gz_interfaces/srv/ControlWorld'], output='screen'),
        bootstrap,
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share / 'launch/ocean.launch.py')),
            launch_arguments={'gui': LaunchConfiguration('gui'), 'world': str(world),
                              'bridge_config': str(share / 'config/landing_bridge.yaml'),
                              'extra_resource_path': str(resources), 'start_paused': 'true'}.items()),
    ]


def generate_launch_description():
    share = Path(get_package_share_directory('ocean_sim'))
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('config', default_value=str(share / 'config/landing.yaml')),
        DeclareLaunchArgument('tag_family', default_value='', description='Override configured family'),
        DeclareLaunchArgument('tag_id', default_value='', description='Override configured marker ID'),
        DeclareLaunchArgument('tag_size', default_value='', description='Outer black-border size in metres'),
        DeclareLaunchArgument('autoland', default_value='true', description='False hovers while detecting tags'),
        OpaqueFunction(function=start),
    ])
