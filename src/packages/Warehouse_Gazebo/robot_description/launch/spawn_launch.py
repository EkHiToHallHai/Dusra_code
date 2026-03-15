from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    model_arg = DeclareLaunchArgument(
        'model', default_value='burger',
        description='model type [burger, waffle, waffle_pi]'
    )
    x_arg = DeclareLaunchArgument('x', default_value='4.52')
    y_arg = DeclareLaunchArgument('y', default_value='2.37')
    z_arg = DeclareLaunchArgument('z', default_value='0.02')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0')
    debug_arg = DeclareLaunchArgument('debug', default_value='false')
    gui_arg = DeclareLaunchArgument('gui', default_value='true')
    pause_arg = DeclareLaunchArgument('pause', default_value='false')
    world_arg = DeclareLaunchArgument('world', default_value='laser')

    set_sensor_rplidar_arg = DeclareLaunchArgument('set_sensor_rplidar', default_value='true')
    set_sensor_lidar_arg = DeclareLaunchArgument('set_sensor_lidar', default_value='false')
    set_sensor_imu_arg = DeclareLaunchArgument('set_sensor_imu', default_value='true')
    set_sensor_camera_arg = DeclareLaunchArgument('set_sensor_camera', default_value='true')

    # Include empty_world.launch from gazebo_ros (ROS 2 version)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'debug': 'false',
            'gui': LaunchConfiguration('gui'),
            'world_name': PathJoinSubstitution([
                FindPackageShare('robot_description'), 'world', [LaunchConfiguration('world'), '.world']
            ]),
        }.items(),
    )

    # Robot description parameter generation by running xacro
    from launch.substitutions import Command
    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([FindPackageShare('robot_description'), 'urdf', 'robot.urdf.xacro']),
        ' ',
        'sensor_rplidar:=', LaunchConfiguration('set_sensor_rplidar'), ' ',
        'sensor_lidar:=', LaunchConfiguration('set_sensor_lidar'), ' ',
        'sensor_imu:=', LaunchConfiguration('set_sensor_imu'), ' ',
        'sensor_camera:=', LaunchConfiguration('set_sensor_camera')
    ])

    robot_description_param = {'robot_description': robot_description_content}
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description_param]
    )

    # Spawn model node equivalent
    spawn_model_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', 'my_robot',
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-Y', LaunchConfiguration('yaw')
        ]
    )

    return LaunchDescription([
        model_arg, x_arg, y_arg, z_arg, yaw_arg, debug_arg, gui_arg, pause_arg, world_arg,
        set_sensor_rplidar_arg, set_sensor_lidar_arg, set_sensor_imu_arg, set_sensor_camera_arg,
        gazebo_launch, robot_state_publisher_node,
        spawn_model_node
    ])
