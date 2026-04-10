import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    use_sim_time = True

    # 1. Safely handle environment variable with a default
    robot_base = os.getenv('LINOROBOT2_BASE', '2wd')

    # Paths
    gazebo_launch_path = PathJoinSubstitution(
        [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']
    )
    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare("linorobot2_base"), "config", "ekf.yaml"]
    )
    urdf_path = PathJoinSubstitution(
        [FindPackageShare("linorobot2_description"),
         "urdf/robots", f"{robot_base}.urdf.xacro"]
    )
    description_launch_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_description'),
         'launch', 'description.launch.py']
    )

    # 2. Build consolidated Gazebo arguments
    # This prevents launching two separate Gazebo processes.
    gz_args = PythonExpression([
        "'-r -s ' + '", LaunchConfiguration('world_path'), "'",
        " + ' -g'" if LaunchConfiguration('gui') == 'true' else " + ''"
    ])

    return LaunchDescription([
        DeclareLaunchArgument(name='gui', default_value='true',
                              description='Enable Gazebo Client'),
        DeclareLaunchArgument(
            name='urdf', default_value=urdf_path, description='URDF path'),
        DeclareLaunchArgument(
            name='world_name', default_value='turtlebot3_world'),
        DeclareLaunchArgument(
            name='world_path',
            default_value=[FindPackageShare(
                'linorobot2_gazebo'), '/worlds/', LaunchConfiguration('world_name'), '.sdf']
        ),
        DeclareLaunchArgument(name='spawn_x', default_value='0.0'),
        DeclareLaunchArgument(name='spawn_y', default_value='0.0'),
        DeclareLaunchArgument(name='spawn_z', default_value='0.0'),
        DeclareLaunchArgument(name='spawn_yaw', default_value='0.0'),

        # 3. Single Gazebo Include
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_path),
            launch_arguments={
                'gz_args': [' -r -s ', LaunchConfiguration('world_path')]
            }.items()
        ),

        # --- Gazebo Client (GUI) ---
        # condition=IfCondition means this only launches if gui:=true
        # Separate from server so GUI can be disabled without stopping simulation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_path),
            condition=IfCondition(LaunchConfiguration('gui')),
            launch_arguments={'gz_args': [' -g']}.items()
        ),

        # 4. Robot Spawner
        Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'linorobot2',
                '-x', LaunchConfiguration('spawn_x'),
                '-y', LaunchConfiguration('spawn_y'),
                '-z', LaunchConfiguration('spawn_z'),
                '-Y', LaunchConfiguration('spawn_yaw'),
            ]
        ),

        # 5. The Bridge
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                "/gps/fix@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat",
                "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
                "/odom/unfiltered@nav_msgs/msg/Odometry[gz.msgs.Odometry",
                "/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU",
                "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
                "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                '/rear_camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                '/rear_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            ],
            # Ensure your bridge remappings match your robot_description frames
            remappings=[
                
            ]
        ),

        # 6. Localization Nodes
        Node(
            package='linorobot2_gazebo',
            executable='gps_to_odom',
            name='gps_to_odom',
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        # Local EKF (Odom -> Base)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',  # Matches YAML header
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}, ekf_config_path],
            remappings=[("odometry/filtered", "odometry/filtered/local")]
        ),

        # Global EKF (Map -> Odom)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_map',  # Matches YAML header
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}, ekf_config_path],
            remappings=[("odometry/filtered", "odometry/filtered/global")]
        ),

        # 7. Robot State Publisher
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch_path),
            launch_arguments={
                'use_sim_time': str(use_sim_time),
                'publish_joints': 'false',
                'urdf': LaunchConfiguration('urdf')
            }.items()
        )
    ])
