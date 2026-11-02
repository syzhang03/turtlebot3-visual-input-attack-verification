"""Combine the existing TurtleBot3 launch and the fixed experiment scenery.

ros_gz_sim create waits for the named world's create service; no fixed sleep.
Reference: https://github.com/gazebosim/ros_gz/blob/jazzy/ros_gz_sim/src/create.cpp
"""
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    EmitEvent, IncludeLaunchDescription, LogInfo, OpaqueFunction,
    RegisterEventHandler, TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    project = Path(__file__).resolve().parents[1]
    scene_file = project / 'worlds' / 'intersection-white-line.sdf'
    tb3 = Path(get_package_share_directory('turtlebot3_gazebo'))
    base_world = tb3 / 'worlds' / 'empty_world.world'
    world = ET.parse(base_world).getroot().find('world')
    if world is None or not world.get('name'):
        raise RuntimeError(f'Cannot determine world name from {base_world}')
    # Validate before starting Gazebo, and preserve the agreed baseline geometry.
    scene = ET.parse(scene_file).getroot()
    for selector in (".//visual[@name='red_box_visual']/pose",
                     ".//collision[@name='red_box_collision']/pose"):
        pose = scene.find(selector)
        if pose is None or [float(v) for v in pose.text.split()] != [4, 0, .25, 0, 0, 0]:
            raise RuntimeError('Expected the baseline red box at X=4.0, Y=0, Z=0.25')

    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(tb3 / 'launch' / 'empty_world.launch.py')),
        launch_arguments={'x_pose': '0.0', 'y_pose': '0.0', 'use_sim_time': 'true'}.items(),
    )
    spawn_scene = Node(
        package='ros_gz_sim', executable='create', name='spawn_experiment_scene',
        output='screen', arguments=[
            '-world', world.get('name'), '-file', str(scene_file),
            '-name', 'boulder_intersection', '-allow_renaming', 'false',
            '-x', '0.0', '-y', '0.0', '-z', '0.0',
        ],
    )
    status = {'loaded': False}

    def on_spawn_exit(event, context):
        if context.is_shutdown:
            return []
        if event.returncode != 0:
            return [LogInfo(msg='SCENE_FAILED: see spawn_experiment_scene output above.'),
                    EmitEvent(event=Shutdown(reason='Experiment scenery creation failed'))]
        status['loaded'] = True
        return [LogInfo(msg='SCENE_READY: scenery loaded. Confirm robot and scene in Gazebo '
                            'before starting a controller. Ctrl+C closes this simulation.')]

    def check_deadline(context):
        if status['loaded'] or context.is_shutdown:
            return []
        return [LogInfo(msg='SCENE_FAILED: scenery did not load within 120 seconds.'),
                EmitEvent(event=Shutdown(reason='Experiment scenery startup timeout'))]

    return LaunchDescription([
        RegisterEventHandler(OnProcessExit(target_action=spawn_scene, on_exit=on_spawn_exit)),
        simulator,
        spawn_scene,
        TimerAction(period=120.0, actions=[OpaqueFunction(function=check_deadline)]),
    ])
