from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ftg_node = Node(
        package='team_planning',
        executable='ftg',
        name='follow_the_gap',
        parameters=[{
            'scan_topic': '/scan',
            'drive_topic': '/drive',
            'use_path_bias': False,
        }],
        output='screen',
    )

    return LaunchDescription([ftg_node])
