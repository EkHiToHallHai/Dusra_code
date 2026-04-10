from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('marker_id',       default_value='0'),
        DeclareLaunchArgument('marker_length',   default_value='0.15'),
        DeclareLaunchArgument('target_distance', default_value='0.05'),
        DeclareLaunchArgument('debug_window',    default_value='false'),
        DeclareLaunchArgument('aruco_dict',      default_value='DICT_5X5_50'),

        Node(
            package='aruco_docking',
            executable='aruco_dock_node',
            name='aruco_dock_node',
            output='screen',
            parameters=[{
                'marker_id':       LaunchConfiguration('marker_id'),
                'marker_length':   LaunchConfiguration('marker_length'),
                'target_distance': LaunchConfiguration('target_distance'),
                'debug_window':    LaunchConfiguration('debug_window'),
                'aruco_dict':      LaunchConfiguration('aruco_dict'),
            }],
        ),
    ])