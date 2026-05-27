"""Launch slam_toolbox + Pure Pursuit + drive mux."""
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

    waypoints_csv = DeclareLaunchArgument(
        'waypoints_csv', description='Path to waypoints CSV file')

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

    pp_node = Node(
        package='team_planning',
        executable='pure_pursuit',
        name='pure_pursuit',
        parameters=[{
            'drive_topic': '/drive',
            'waypoints_csv': LaunchConfiguration('waypoints_csv'),
            'use_slam_pose': True,
            'map_frame': 'map',
            'base_frame': 'ego_racecar/base_link',
        }],
        output='screen',
    )

    return LaunchDescription([
        use_sim_time,
        slam_params_file,
        waypoints_csv,
        static_tf,
        slam_node,
        pp_node,
    ])
