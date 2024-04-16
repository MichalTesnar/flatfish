import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('ros2_canbus'),
        'config',
        'example.yaml'
        )
        
    node=Node(
        package = 'ros2_canbus',
        name = 'ros2_canbus_node',
        executable = 'ros2_canbus_node',
        parameters = [config]
    )
    ld.add_action(node)
    return ld