import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pressure_config_file=os.path.join(
        get_package_share_directory('pressure_pkg'),
        'config',
        'pressure.yaml',
    )

    return LaunchDescription([
        Node(
            package='pressure_pkg',
            executable='pressure_node',
            output='screen',
            parameters=[pressure_config_file],
            # name='pressure_node',
        )
    ])