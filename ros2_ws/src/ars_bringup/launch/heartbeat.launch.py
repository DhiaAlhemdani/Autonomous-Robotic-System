from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='ars_bringup', executable='heartbeat', name='heartbeat', output='screen'),
    ])
