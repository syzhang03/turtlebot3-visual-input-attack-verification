from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = str(Path(get_package_share_directory('tb3_camera_attack'))
                 / 'config' / 'camera_occlusion.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=config),
        DeclareLaunchArgument('attack_enabled', default_value='false'),
        Node(
            package='tb3_camera_attack', executable='camera_occlusion',
            name='camera_occlusion', output='screen',
            parameters=[LaunchConfiguration('params_file'), {
                'attack_enabled': ParameterValue(
                    LaunchConfiguration('attack_enabled'), value_type=bool),
            }],
        ),
    ])
