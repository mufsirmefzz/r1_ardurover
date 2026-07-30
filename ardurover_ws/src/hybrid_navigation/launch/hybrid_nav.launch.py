import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('hybrid_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    twist_mux_params = os.path.join(pkg_dir, 'config', 'twist_mux_params.yaml')
    ekf_params = os.path.join(pkg_dir, 'config', 'ekf.yaml')
    nav2_params = os.path.join(pkg_dir, 'config', 'nav2_local.yaml')

    return LaunchDescription([
        # 1. Local Odometry Fusion
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_params]
        ),

        # 2. Twist Mux (Command Multiplexer)
        Node(
            package='twist_mux',
            executable='twist_mux',
            output='screen',
            parameters=[twist_mux_params],
            remappings=[('/cmd_vel_out', '/cmd_vel')]
        ),

        # 3. Nav2 (controller, planner, behaviors, bt_navigator — no AMCL)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'false',
                'params_file': nav2_params,
                'autostart': 'true',
            }.items()
        ),

        # 4. Hybrid Executive Node
        # BUG #16 FIX: executable= must be the console_scripts key ('hybrid_execution'),
        # NOT the Python source filename ('hybrid_executive.py').
        Node(
            package='hybrid_navigation',
            executable='hybrid_execution',
            name='hybrid_executive',
            output='screen'
        ),
        
        # 5. Static Transform Publisher (base_link -> laser)
        # Required because Nav2 needs to know where the LiDAR is mounted relative to the robot center.
        # Without this, tf2_ros::MessageFilter will drop all /scan messages with a misleading timestamp error.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser',
            arguments=['0', '0', '0.2', '3.14159', '0', '0', 'base_link', 'laser']
        )
    ])
