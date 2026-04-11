#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    ExecuteProcess,
    SetEnvironmentVariable
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    robot_base = os.getenv('LINOROBOT2_BASE', 'hamo_bot')

    gazebo_launch_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_gazebo'), 'launch', 'gazebo.launch.py']
    )

    return LaunchDescription([

        # ── Launch Arguments ──────────────────────────────────────────────────

        DeclareLaunchArgument(
            name='world_name',
            default_value='baylands_world',
            description='Gazebo world name (without .sdf)'
        ),
        DeclareLaunchArgument(
            name='spawn_x',
            default_value='0.0',
            description='Robot spawn X position'
        ),
        DeclareLaunchArgument(
            name='spawn_y',
            default_value='0.0',
            description='Robot spawn Y position'
        ),
        DeclareLaunchArgument(
            name='spawn_z',
            default_value='0.0',
            description='Robot spawn Z position'
        ),
        DeclareLaunchArgument(
            name='spawn_yaw',
            default_value='0.0',
            description='Robot spawn yaw orientation'
        ),

        # ── 1. Launch Gazebo (server + client + robot + bridges + EKF) ───────
        # This calls linorobot2_gazebo/launch/gazebo.launch.py which internally
        # handles: gz server, gz client, robot_state_publisher, parameter_bridge,
        # gps_to_odom, ekf_node (local), ekf_node (global)

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_path),
            launch_arguments={
                'world_name': LaunchConfiguration('world_name'),
                'spawn_x':    LaunchConfiguration('spawn_x'),
                'spawn_y':    LaunchConfiguration('spawn_y'),
                'spawn_z':    LaunchConfiguration('spawn_z'),
                'spawn_yaw':  LaunchConfiguration('spawn_yaw'),
            }.items()
        ),

        # ── 2. CLF-CBF Controller Service Node ───────────────────────────────
        # Waits 10 seconds for Gazebo + EKF to fully initialize before starting.
        # Provides the /dr_clf_cbf_controller/compute_twist service.
        # The controller uses the lidar scan for CBF-based obstacle avoidance
        # and the CLF for goal convergence.

        TimerAction(
            period=20.0,
            actions=[
                Node(
                    package='dr_clf_cbf_controller',
                    executable='dr_cbf_controller_node',
                    name='clf_cbf_controller',
                    output='screen',
                    parameters=[{'use_sim_time': True}]
                )
            ]
        ),

        # ── 3. Waypoint Navigator Node ────────────────────────────────────────
        # Waits 14 seconds to ensure CLF-CBF service is up and accepting requests.
        # Reads lat/lon waypoints, converts to XY using the same origin as
        # gps_to_odom, then calls the CLF-CBF service at 10Hz to get cmd_vel
        # and publishes to /cmd_vel.
        # Advances to next waypoint when within GOAL_RADIUS metres of target.

        TimerAction(
            period=24.0,
            actions=[
                Node(
                    package='dr_clf_cbf_controller',
                    executable='waypoint_navigator',
                    name='waypoint_navigator',
                    output='screen',
                    parameters=[{'use_sim_time': True}]
                )
            ]
        ),

    ])