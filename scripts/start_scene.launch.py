"""Combine the TurtleBot3 launch with either experiment scene.

``scene:=full`` loads the fixed red obstacle at X=4.0.  ``scene:=empty`` loads
the same road, houses and white stop line without any obstacle ahead.  Both
scenes are spawned into the installed empty world, and ``ros_gz_sim create``
waits for the world's create service instead of using a fixed sleep.

Reference: https://github.com/gazebosim/ros_gz/blob/jazzy/ros_gz_sim/src/create.cpp
"""
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, LogInfo,
    OpaqueFunction, RegisterEventHandler, TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

SCENES = {
    'full': ('intersection-white-line.sdf', True),
    'empty': ('empty-road-white-line.sdf', False),
}


def _pose_matches(element, expected, tolerance=1e-6):
    if element is None or element.text is None:
        return False
    try:
        values = [float(value) for value in element.text.split()]
    except ValueError:
        return False
    return len(values) == len(expected) and all(
        abs(actual - wanted) <= tolerance
        for actual, wanted in zip(values, expected))


def _validate_scene(scene_file, expect_red):
    if not scene_file.is_file():
        raise RuntimeError(f'Missing scene file: {scene_file}')
    scene = ET.parse(scene_file).getroot()
    for name in ('road_x_visual', 'stop_line_visual'):
        if scene.find(f".//visual[@name='{name}']") is None:
            raise RuntimeError(f'Scene is missing {name}: {scene_file}')
    red_visual = scene.find(".//visual[@name='red_box_visual']")
    red_collision = scene.find(".//collision[@name='red_box_collision']")
    if expect_red:
        expected = [4, 0, .25, 0, 0, 0]
        visual_ok = _pose_matches(
            red_visual.find('pose') if red_visual is not None else None, expected)
        collision_ok = _pose_matches(
            red_collision.find('pose') if red_collision is not None else None, expected)
        if not (visual_ok and collision_ok):
            raise RuntimeError('Expected the baseline red box at X=4.0, Y=0, Z=0.25')
    elif red_visual is not None or red_collision is not None:
        raise RuntimeError(f'Empty scene must not contain the red obstacle: {scene_file}')
    return scene


def _make_scene_actions(context):
    scene_name = context.launch_configurations.get('scene', 'full')
    if scene_name not in SCENES:
        return [
            LogInfo(msg=f"SCENE_FAILED: unknown scene '{scene_name}'. Use full or empty."),
            EmitEvent(event=Shutdown(reason='Unknown experiment scene')),
        ]
    filename, expect_red = SCENES[scene_name]
    project = Path(__file__).resolve().parents[1]
    scene_file = project / 'worlds' / filename
    tb3 = Path(get_package_share_directory('turtlebot3_gazebo'))
    base_world = tb3 / 'worlds' / 'empty_world.world'
    world = ET.parse(base_world).getroot().find('world')
    if world is None or not world.get('name'):
        raise RuntimeError(f'Cannot determine world name from {base_world}')
    try:
        _validate_scene(scene_file, expect_red)
    except (OSError, ET.ParseError, RuntimeError) as exc:
        return [
            LogInfo(msg=f'SCENE_FAILED: {exc}'),
            EmitEvent(event=Shutdown(reason='Experiment scenery validation failed')),
        ]

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

    def on_spawn_exit(event, context_):
        if context_.is_shutdown:
            return []
        if event.returncode != 0:
            return [LogInfo(msg='SCENE_FAILED: see spawn_experiment_scene output above.'),
                    EmitEvent(event=Shutdown(reason='Experiment scenery creation failed'))]
        status['loaded'] = True
        label = 'red obstacle present' if expect_red else 'no obstacle ahead'
        return [LogInfo(msg=f'SCENE_READY: {scene_name} scenery loaded ({label}). '
                            'Confirm robot and scene in Gazebo before starting a controller. '
                            'Ctrl+C closes this simulation.')]

    def check_deadline(context_):
        if status['loaded'] or context_.is_shutdown:
            return []
        return [LogInfo(msg='SCENE_FAILED: scenery did not load within 120 seconds.'),
                EmitEvent(event=Shutdown(reason='Experiment scenery startup timeout'))]

    return [
        RegisterEventHandler(OnProcessExit(target_action=spawn_scene, on_exit=on_spawn_exit)),
        simulator,
        spawn_scene,
        TimerAction(period=120.0, actions=[OpaqueFunction(function=check_deadline)]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'scene', default_value='full',
            description="Experiment scene: 'full' (red obstacle) or 'empty' (no obstacle ahead)"),
        OpaqueFunction(function=_make_scene_actions),
    ])
