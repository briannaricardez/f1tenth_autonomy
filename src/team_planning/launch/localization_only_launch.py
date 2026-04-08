"""Launch slam_toolbox in localization mode only."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('team_planning')
    default_params = os.path.join(
        pkg_share, 'config', 'slam_toolbox_localization_params.yaml'
    )

    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true'
    )

    slam_params_file = DeclareLaunchArgument(
        'slam_params_file',
        default_value=default_params
    )

    slam_node = Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            LaunchConfiguration('slam_params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    return LaunchDescription([
        use_sim_time,
        slam_params_file,
        slam_node,
    ])
