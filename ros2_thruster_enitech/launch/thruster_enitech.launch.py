import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('thruster_enitech'),
        'config',
        'thruster_enitech_node.yaml'
        )
        
    node=Node(
        package = 'thruster_enitech',
        name = 'thruster_enitech_node',
        executable = 'thruster_enitech_node',
        parameters = [config],
        remappings=[('/thruster_heave_front/can_in', '/ros2_canbus_node/rx_unwatched_message'),
                    ('/thruster_heave_front/can_out', '/ros2_canbus_node/tx_message'),
                    ('/thruster_heave_front/CommandWrenchStamped', '/ThrustersJointCommand')]
    )
    ld.add_action(node)
    return ld
