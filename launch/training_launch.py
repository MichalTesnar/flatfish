from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mystery',
            executable='inference',
            name='inference'
        ),
        Node(
            package='mystery',
            executable='training',
            name='training'
        ),
    ])