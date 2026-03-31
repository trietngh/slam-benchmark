import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_share = get_package_share_directory('frame_saver')
    config_file = os.path.join(pkg_share, 'config', 'frame_saver.yaml')

    return LaunchDescription([
        # Topics and encodings are now configured exclusively via frame_saver.yaml
        DeclareLaunchArgument(
            'save_rate',
            default_value='0.0',
            description='Rate to save images (Hz). 0 for all (async) or timer frequency (sync)'),
        DeclareLaunchArgument(
            'save_dir',
            default_value='.',
            description='Directory to save images'),

        DeclareLaunchArgument(
            'mode',
            default_value='async',
            description='Mode: "async" or "sync"'),
        DeclareLaunchArgument(
            'timeout',
            default_value='0.1',
            description='Timeout for sync mode (seconds)'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'),

        Node(
            package='frame_saver',
            executable='frame_saver_node',
            name='frame_saver',
            output='screen',
            parameters=[config_file]
        )
    ])
