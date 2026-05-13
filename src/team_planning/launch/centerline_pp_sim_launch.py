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
            'use_slam_pose': True,
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
        use_sim_time,
        static_tf,
        centerline_node,
        pp_node,
        ftg_node,
        supervisor_node,
        mux_node,
    ])
