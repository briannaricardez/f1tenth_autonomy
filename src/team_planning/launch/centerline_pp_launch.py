"""Map-free racing: VESC + LiDAR + centerline follower + Pure Pursuit + FTG safety.

No SLAM, no map_server, no waypoint CSV. The centerline follower extracts a
rolling local path from each LiDAR scan; Pure Pursuit runs that path in the
ego_racecar/base_link frame; the drive arbiter continuously blends PP and FTG
based on minimum front distance.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('team_planning')
    default_vesc_params = os.path.expanduser(
        '~/f1tenth_autonomy/src/vesc/vesc_driver/params/vesc_config.yaml'
    )
    default_urg_params = os.path.join(pkg_share, 'config', 'urg_node_params.yaml')

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

    centerline_node = Node(
        package='team_planning',
        executable='centerline_follower',
        name='centerline_follower',
        output='screen',
        parameters=[{
            'scan_topic': '/scan',
            'local_path_topic': '/local_path',
            'output_frame': 'ego_racecar/base_link',
            'forward_fov_deg': 180.0,
            'lookahead_samples': [0.3, 0.6, 0.9, 1.2, 1.5],
            'slab_half_width': 0.15,
            'range_min': 0.15,
            'range_max': 10.0,
            'smoothing_window': 5,
            'publish_markers': True,
        }],
    )

    pp_node = Node(
        package='team_planning',
        executable='pure_pursuit',
        name='pure_pursuit',
        output='screen',
        parameters=[{
            'drive_topic': '/drive_pp',
            'waypoints_csv': '',
            'use_slam_pose': False,
            'map_frame': 'ego_racecar/base_link',
            'base_frame': 'ego_racecar/base_link',
            'use_local_path_topic': True,
            'local_path_topic': '/local_path',
            'lookahead_distance': 1.0,
            'curvature_scale': 1.5,
            'max_speed': 4.0,
            'min_speed': 0.8,
            'wheelbase': 0.33,
            'max_steering_angle': 0.36,
            'loop_path': False,
        }],
    )

    ftg_node = Node(
        package='team_planning',
        executable='ftg',
        name='follow_the_gap',
        output='screen',
        parameters=[{
            'scan_topic': '/scan',
            'drive_topic': '/drive_ftg',
            'max_speed': 2.5,
            'min_speed': 0.5,
            'max_steer': 0.35,
            'bubble_radius': 0.4,
            'gap_threshold': 0.8,
            'min_range': 0.15,
            'steer_slew_rate': 5.0,
            'front_danger_dist': 0.8,
            'front_danger_angle_deg': 15.0,
            'local_path_topic': '/local_path',
            'use_path_bias': True,
            'path_lookahead_dist': 1.0,
            'path_stale_timeout': 0.5,
            'path_bias_weight': 1.5,
            'gap_size_weight': 0.5,
            'gap_range_weight': 1.0,
            'front_danger_gain': 2.0,
        }],
    )

    arbiter_node = Node(
        package='team_control',
        executable='drive_arbiter',
        name='drive_arbiter',
        output='screen',
        parameters=[{
            'pp_topic': '/drive_pp',
            'ftg_topic': '/drive_ftg',
            'scan_topic': '/scan',
            'out_topic': '/drive',
            'blend_topic': '/control_blend',
            'arbiter_rate': 50.0,
            'msg_stale_timeout': 0.2,
            'scan_stale_timeout': 0.5,
            'blend_far': 1.5,
            'blend_near': 0.5,
            'front_half_angle_deg': 10.0,
            'alpha_smooth_beta': 0.4,
            'max_speed': 4.0,
            'emergency_speed': 0.4,
            'beta_low': 0.25,
            'beta_high': 0.7,
            'slew_low': 2.5,
            'slew_high': 6.0,
            'v_ref': 2.0,
        }],
    )

    return LaunchDescription([
        vesc_driver,
        lidar_driver,
        ackermann_bridge,
        vesc_to_odom,
        static_tf,
        centerline_node,
        pp_node,
        TimerAction(period=1.0, actions=[ftg_node]),
        arbiter_node,
    ])
