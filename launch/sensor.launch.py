from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pressure_pkg',
            executable='pressure_node',
            output='screen',
        ),
    ])
