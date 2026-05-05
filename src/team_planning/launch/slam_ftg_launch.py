"""Launch VESC + LiDAR + slam_toolbox (mapping) + FTG for real hardware map building."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import TimerAction

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
        'use_sim_time', default_value='false')  # FALSE for real hardware

    slam_params_file = DeclareLaunchArgument(
        'slam_params_file', default_value=default_params)

    # ── Hardware drivers ──────────────────────────────────────────────────────

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

    # ── TF ───────────────────────────────────────────────────────────────────

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=['0', '0', '0', '0', '0', '0',
                   'ego_racecar/base_link', 'ego_racecar/laser'],
    )

    # ── SLAM (mapping mode) ───────────────────────────────────────────────────

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

    # ── FTG (drives the car during mapping) ──────────────────────────────────

    ftg_node = Node(
        package='team_planning',
        executable='ftg',
        name='follow_the_gap',
        output='screen',
        parameters=[{
            'scan_topic': '/scan',
            'drive_topic': '/drive',
            'max_speed': 1.0,   # slow and safe for mapping
            'min_speed': 0.5,
        }],
    )

    return LaunchDescription([
        use_sim_time,
        slam_params_file,
        # Hardware first
        vesc_driver,
        lidar_driver,
        ackermann_bridge,
        vesc_to_odom,
        # TF and SLAM
        static_tf,
        slam_node,
        # FTG to drive during mapping
        TimerAction(period=5.0, actions=[ftg_node]),
    ])
