"""Localization + MPP + PP + FTG + automatic safety supervisor + drive mux."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('team_planning')
    default_params = os.path.join(
        pkg_share, 'config', 'slam_toolbox_localization_params.yaml'
    )

    use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true')

    slam_params_file = DeclareLaunchArgument(
        'slam_params_file', default_value=default_params)

    waypoints_csv = DeclareLaunchArgument(
        'waypoints_csv',
        description='Path to waypoints CSV file'
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
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            LaunchConfiguration('slam_params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
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
        slam_params_file,
        waypoints_csv,
        static_tf,
        slam_node,
        mpp_node,
        pp_node,
        ftg_node,
        supervisor_node,
        mux_node,
    ])
