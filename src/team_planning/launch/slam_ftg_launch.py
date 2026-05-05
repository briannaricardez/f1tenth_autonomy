"""Launch VESC + LiDAR + slam_toolbox (mapping) + FTG for real hardware map building."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('team_planning')
    default_params = os.path.join(pkg_share, 'config', 'slam_toolbox_params.yaml')
    default_vesc_params = os.path.expanduser(
        '~/f1tenth_autonomy/src/vesc/vesc_driver/params/vesc_config.yaml'
    )
    default_urg_params = (
        '/opt/ros/humble/share/urg_node/launch/urg_node_ethernet.yaml'
    )

    use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false')

    slam_params_file = DeclareLaunchArgument(
        'slam_params_file', default_value=default_params)

    vesc_driver = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver',
        output='screen',
        parameters=[default_vesc_params],
    )

    lidar_driver = Node(
        package='urg_node',
        executable='urg_node_driver',
        name='urg_node',
        output='screen',
        parameters=[default_urg_params],
    )

    ackermann_bridge = Node(
        package='vesc_ackermann',
        executable='ackermann_to_vesc_node',
        name='ackermann_to_vesc',
        output='screen',
        parameters=[{
            'speed_to_erpm_gain': 4614.0,
            'speed_to_erpm_offset': 0.0,
            'steering_angle_to_servo_gain': -1.2135,
            'steering_angle_to_servo_offset': 0.5530,
        }],
        remappings=[('/ackermann_cmd', '/drive')],
    )

    vesc_to_odom = Node(
        package='vesc_ackermann',
        executable='vesc_to_odom_node',
        name='vesc_to_odom',
        output='screen',
        parameters=[{
            'speed_to_erpm_gain': 4614.0,
            'speed_to_erpm_offset': 0.0,
            'steering_angle_to_servo_gain': -1.2135,
            'steering_angle_to_servo_offset': 0.5530,
            'wheelbase': 0.33,
            'odom_frame': 'ego_racecar/odom',
            'base_frame': 'ego_racecar/base_link',
            'publish_tf': True,
        }],
    )

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
        output='screen',
        parameters=[{
            'scan_topic': '/scan',
            'drive_topic': '/drive',
            'max_speed': 0.7,
            'min_speed': 0.3,
            'max_steer': 0.35,
            'bubble_radius': 0.6,
            'gap_threshold': 0.8,
            'min_range': 0.15,
            'steer_slew_rate': 5.0,
            'front_danger_dist': 0.8,
            'front_danger_angle_deg': 30.0,
        }],
    )

    return LaunchDescription([
        use_sim_time,
        slam_params_file,
        vesc_driver,
        lidar_driver,
        ackermann_bridge,
        vesc_to_odom,
        static_tf,
        slam_node,
        TimerAction(period=5.0, actions=[ftg_node]),
    ])
