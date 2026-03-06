from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
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

    return LaunchDescription([ftg_node, mux_node])
