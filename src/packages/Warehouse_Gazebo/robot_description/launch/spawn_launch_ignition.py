from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import TextSubstitution,LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare launch arguments
    model_arg = DeclareLaunchArgument(
        'model', default_value='burger', description='model type [burger, waffle, waffle_pi]'
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

    # Paths
    robot_description_share = get_package_share_directory('robot_description')
    robot_description_path = os.path.join(robot_description_share, 'urdf', 'robot.urdf.xacro')
    world_path = PathJoinSubstitution([
        robot_description_share,
        'world',
        [LaunchConfiguration('world'), '.world']
    ])

    # Generate robot description from xacro — typical approach is to use a function or external node,
    # here we assume the robot_description parameter is set by some external node or process.
    # Alternatively, use xacro in a command substitution with ExecuteProcess if preferred.

    # Include gazebo launch file replacement using ros_gz packages
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': ['-r ', world_path],
            'gui': LaunchConfiguration('gui'),
            'pause': LaunchConfiguration('pause'),
            'use_sim_time': 'true'
        }.items(),
    )

    # Spawn the robot model (using ros_gz spawn entity node)
    spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_entity',
        arguments=[
            '-file', robot_description_path,
            '-name', LaunchConfiguration('model'),
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-Y', LaunchConfiguration('yaw')
        ],
        output='screen'
    )

    return LaunchDescription([
        model_arg, x_arg, y_arg, z_arg, yaw_arg, debug_arg, gui_arg, pause_arg, world_arg,
        set_sensor_rplidar_arg, set_sensor_lidar_arg, set_sensor_imu_arg, set_sensor_camera_arg,
        gazebo_launch,
        spawn_entity_node,
    ])
