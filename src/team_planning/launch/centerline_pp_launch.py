"""Map-free racing: VESC + LiDAR + centerline follower + Pure Pursuit + FTG safety.

No SLAM, no map_server, no waypoint CSV. The centerline follower extracts a
rolling local path from each LiDAR scan; Pure Pursuit runs that path in the
ego_racecar/base_link frame; the safety supervisor switches to FTG on close
obstacles via drive_mux.
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
            'lookahead_samples': [0.4, 0.8, 1.2, 1.6, 2.0, 2.5, 3.0],
            'slab_half_width': 0.20,
            'range_min': 0.15,
            'range_max': 10.0,
            'smoothing_window': 3,
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
            'use_slam_pose': True,
            'map_frame': 'ego_racecar/base_link',
            'base_frame': 'ego_racecar/base_link',
            'use_local_path_topic': True,
            'local_path_topic': '/local_path',
            'lookahead_distance': 1.2,
            'max_speed': 1.5,
            'min_speed': 0.5,
            'wheelbase': 0.33,
            'max_steering_angle': 0.4189,
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

    supervisor_node = Node(
        package='team_control',
        executable='safety_supervisor',
        name='safety_supervisor',
        output='screen',
        parameters=[{
            'scan_topic': '/scan',
            'mode_topic': '/control_mode',
            'ftg_trigger_dist': 0.8,
            'pp_return_dist': 1.0,
            'front_half_angle_deg': 20.0,
            'default_mode': 'pp',
        }],
    )

    mux_node = Node(
        package='team_control',
        executable='drive_mux',
        name='drive_mux',
        output='screen',
        parameters=[{
            'pp_topic': '/drive_pp',
            'ftg_topic': '/drive_ftg',
            'mode_topic': '/control_mode',
            'out_topic': '/drive',
            'default_mode': 'pp',
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
        TimerAction(period=5.0, actions=[ftg_node]),
        supervisor_node,
        mux_node,
    ])
