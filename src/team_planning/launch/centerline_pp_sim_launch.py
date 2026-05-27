"""Map-free racing in simulation: centerline follower + Pure Pursuit + FTG safety.

Assumes f1tenth_gym_ros bridge is already running (provides /scan, odom, TFs).
No SLAM, no map, no waypoint CSV.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')

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
            'use_slam_pose': False,
            'map_frame': 'ego_racecar/base_link',
            'base_frame': 'ego_racecar/base_link',
            'use_local_path_topic': True,
            'local_path_topic': '/local_path',
            'lookahead_distance': 1.2,
            'max_speed': 2.0,
            'min_speed': 0.8,
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
            'max_speed': 1.0,
            'min_speed': 0.4,
            'max_steer': 0.35,
            'bubble_radius': 0.4,
            'gap_threshold': 0.8,
            'min_range': 0.15,
            'steer_slew_rate': 5.0,
            'front_danger_dist': 0.8,
            'front_danger_angle_deg': 30.0,
            'local_path_topic': '/local_path',
            'use_path_bias': True,
            'path_lookahead_dist': 1.2,
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
            'front_half_angle_deg': 20.0,
            'alpha_smooth_beta': 0.4,
            'max_speed': 2.0,
            'emergency_speed': 0.4,
            'beta_low': 0.25,
            'beta_high': 0.7,
            'slew_low': 2.5,
            'slew_high': 6.0,
            'v_ref': 2.0,
        }],
    )

    return LaunchDescription([
        use_sim_time,
        static_tf,
        centerline_node,
        pp_node,
        ftg_node,
        arbiter_node,
    ])
