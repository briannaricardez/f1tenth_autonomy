"""Localization + MPP + PP + FTG + automatic safety supervisor + drive mux."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('team_planning')

    amcl_params = os.path.join(pkg_share, 'config', 'amcl_params.yaml')
    map_server_params = os.path.join(pkg_share, 'config', 'map_server_params.yaml')

    use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true')

    waypoints_csv = DeclareLaunchArgument(
        'waypoints_csv',
        description='Path to waypoints CSV file'
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[map_server_params],
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_params],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': True,
            'bond_timeout': 0.0,
            'node_names': ['map_server', 'amcl'],
        }],
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=['0', '0', '0', '0', '0', '0',
                   'ego_racecar/base_link', 'ego_racecar/laser'],
    )

    mpp_node = Node(
        package='team_planning',
        executable='mpp',
        name='mpp',
        output='screen',
        parameters=[{
            'waypoints_csv': LaunchConfiguration('waypoints_csv'),
            'path_topic': '/local_path',
            'scan_topic': '/scan',
            'map_frame': 'map',
            'base_frame': 'ego_racecar/base_link',
            'horizon_points': 25,
            'publish_rate': 15.0,
            'loop_path': True,
        }],
    )

    pp_node = Node(
        package='team_planning',
        executable='pure_pursuit',
        name='pure_pursuit',
        output='screen',
        parameters=[{
            'drive_topic': '/drive_pp',
            'waypoints_csv': LaunchConfiguration('waypoints_csv'),
            'use_slam_pose': True,
            'map_frame': 'map',
            'base_frame': 'ego_racecar/base_link',
            'use_local_path_topic': True,
            'local_path_topic': '/local_path',
            'lookahead_distance': 1.2,
            'max_speed': 3.5,
            'min_speed': 1.5,
            'curvature_lookahead_points': 8,
            'wheelbase': 0.33,
            'max_steering_angle': 0.4189,
            'loop_path': True,
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
            'max_speed': 1.4,
            'min_speed': 1.0,
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
            'ftg_trigger_dist': 1.0,
            'pp_return_dist': 1.3,
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
        waypoints_csv,
        static_tf,
        map_server_node,
        amcl_node,
        lifecycle_manager,
        mpp_node,
        pp_node,
        ftg_node,
        supervisor_node,
        mux_node,
    ])
