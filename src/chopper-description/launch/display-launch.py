from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='chopper_brain',
            executable='spektrum_node',
            name='spektrum_bridge'
        )
    ])
