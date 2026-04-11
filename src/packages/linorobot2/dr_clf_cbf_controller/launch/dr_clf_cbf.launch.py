from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dr_clf_cbf_controller',
            executable='dr_cbf_controller',
            name='clf_cbf_controller',
            output='screen',
        )
    ])