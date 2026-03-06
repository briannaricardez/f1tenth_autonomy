"""Launch slam_toolbox + Follow-The-Gap + drive mux."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('team_planning')
    default_params = os.path.join(pkg_share, 'config', 'slam_toolbox_params.yaml')

    use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true')

    slam_params_file = DeclareLaunchArgument(
        'slam_params_file', default_value=default_params)

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=['0', '0', '0', '0', '0', '0',
                   'ego_racecar/base_link', 'ego_racecar/laser'],
    )

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            LaunchConfiguration('slam_params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    ftg_node = Node(
        package='team_planning',
        executable='ftg',
        name='follow_the_gap',
        parameters=[{
            'scan_topic': '/scan',
            'drive_topic': '/drive_ftg',
        }],
        output='screen',
    )

    mux_node = Node(
        package='team_control',
        executable='drive_mux',
        name='drive_mux',
        parameters=[{
            'in_topic': '/drive_ftg',
            'out_topic': '/drive',
        }],
        output='screen',
    )

    return LaunchDescription([
        use_sim_time,
        slam_params_file,
        static_tf,
        slam_node,
        ftg_node,
        mux_node,
    ])
