from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import FindPackageShare

import os

def generate_launch_description():
    # Paths to YAML config files - use FindPackageShare and os.path.join to locate them
    robot_control_share = FindPackageShare('robot_control').find('robot_control')
    config_yaml = os.path.join(robot_control_share, 'config', 'config.yaml')
    diff_drive_yaml = os.path.join(robot_control_share, 'config', 'diff_drive.yaml')

    return LaunchDescription([

        # Lifting tray controller - load YAML params via parameters argument in Node
        Node(
            package='controller_manager',
            executable='spawner',
            name='controller_spawner',
            namespace='my_robot',
            output='screen',
            arguments=['joint_state_controller', 'joint1_position_controller'],
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace='',
            output='screen',
            remappings=[('/joint_states', '/my_robot/joint_states')],
            parameters=[config_yaml],
        ),

        # Teleop control node with parameters and remapping
        Node(
            package='robot_control',
            executable='laser_teleop_keyboard',
            name='teleop_keyboard',
            output='screen',
            parameters=[diff_drive_yaml, 
                        {'scale_linear': 0.5, 'scale_angular': 1.5}],
            remappings=[('teleop_keyboard/cmd_vel', '/cmd_vel')],
        ),

    ])
