"""Two relays in every condition; robot control is started separately after warmup."""
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import subprocess

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from tb3_lidar_attack.experiment import condition


def setup(context):
    args = {key: LaunchConfiguration(key).perform(context) for key in (
        'mode', 'camera_delay_s', 'lidar_bias_m', 'sector_half_width_deg', 'duration_s')}
    manifest = condition(**args)
    effective = manifest['effective']
    run_id = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
    root = Path(LaunchConfiguration('results_dir').perform(context)).expanduser().resolve()
    run_dir = root / run_id
    run_dir.mkdir(parents=True, exist_ok=False)
    project = Path(__file__).resolve().parents[1]
    try:
        revision = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=project,
                                           stderr=subprocess.DEVNULL, text=True).strip()
        dirty = bool(subprocess.check_output(['git', 'status', '--porcelain'], cwd=project,
                                             stderr=subprocess.DEVNULL, text=True).strip())
    except (OSError, subprocess.CalledProcessError):
        revision, dirty = 'source bundle; see SOURCE_REVISION.txt if supplied', None
    manifest.update(run_id=run_id, ros_domain_id=os.environ.get('ROS_DOMAIN_ID', '0'),
                    source_revision=revision, source_dirty=dirty,
                    use_sim_time=True, camera_header_policy='current header, historical payload',
                    camera_transport=dict(input_reliability='reliable', output_reliability='reliable',
                                          input_depth=1, output_depth=1,
                                          revision='joint-reliable-camera-input-v2'),
                    lidar_header_policy='unchanged',
                    topics=dict(camera_input='/camera/image_raw', camera_output='/attack/image_raw',
                                lidar_input='/scan', lidar_output='/attack/scan'))
    (run_dir / 'manifest.json').write_text(json.dumps(manifest, indent=2), encoding='utf-8')
    pointer = root / f'.latest-{run_id}.tmp'
    pointer.write_text(str(run_dir), encoding='utf-8')
    pointer.replace(root / 'latest-run.txt')
    camera = Node(package='tb3_visual_baseline', executable='delay_replay', output='screen', parameters=[dict(
        use_sim_time=True, input_topic='/camera/image_raw', output_topic='/attack/image_raw',
        input_reliability='reliable', output_reliability='reliable',
        delay_s=effective['camera_delay_s'], start_s=effective['start_s'],
        duration_s=effective['duration_s'], log_dir=str(run_dir / 'camera'))])
    lidar = Node(package='tb3_lidar_attack', executable='lidar_bias', output='screen', parameters=[dict(
        use_sim_time=True, input_topic='/scan', output_topic='/attack/scan', output_reliability='reliable',
        bias_m=effective['lidar_bias_m'], sector_center_deg=effective['sector_center_deg'],
        sector_half_width_deg=effective['sector_half_width_deg'], start_s=effective['start_s'],
        duration_s=effective['duration_s'], log_dir=str(run_dir / 'lidar'))])
    return [
        LogInfo(msg=f'Joint experiment: {run_dir}; mode={manifest["mode"]}; parameters={effective}'),
        RegisterEventHandler(OnProcessExit(target_action=camera,
            on_exit=[EmitEvent(event=Shutdown(reason='Camera relay exited; stopping both relays'))])),
        RegisterEventHandler(OnProcessExit(target_action=lidar,
            on_exit=[EmitEvent(event=Shutdown(reason='LiDAR relay exited; stopping both relays'))])),
        camera, lidar,
    ]


def generate_launch_description():
    defaults = dict(mode='clean', camera_delay_s='1.0', lidar_bias_m='1.0',
                    sector_half_width_deg='30.0', duration_s='600.0',
                    results_dir='~/joint-attack-results')
    return LaunchDescription([DeclareLaunchArgument(key, default_value=value)
                              for key, value in defaults.items()] + [OpaqueFunction(function=setup)])
