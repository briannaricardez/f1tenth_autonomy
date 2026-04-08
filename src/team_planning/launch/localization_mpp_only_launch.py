"""Launch slam_toolbox in localization mode + MPP only."""
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

    waypoints_csv = DeclareLaunchArgument(
        'waypoints_csv',
        description='Path to waypoints CSV file'
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

    mpp_node = Node(
        package='team_planning',
        executable='mpp',
        name='mpp',
        output='screen',
        parameters=[{
            'waypoints_csv': LaunchConfiguration('waypoints_csv'),
            'path_topic': '/local_path',
            'scan_topic': '/scan',
            'map_frame': 'map',
            'base_frame': 'ego_racecar/base_link',
            'horizon_points': 25,
            'publish_rate': 15.0,
            'loop_path': True,
        }],
    )

    return LaunchDescription([
        use_sim_time,
        slam_params_file,
        waypoints_csv,
        slam_node,
        mpp_node,
    ])
