from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        #Node(
        #    package='ros2bag',
        #    executable='play',
        #    name='player',
        #    #output='screen',
        #    arguments=['/home/michal/ros2_ws/src/rosbags/rosbag2_long_gathered']
        #),
        Node(
            package='mystery',
            executable='inferencer',
            name='inferencer'
        ),
        Node(
            package='mystery',
            executable='trainer',
            name='trainer'
        ),
        Node(
            package='mystery',
            executable='evaluator',
            name='evaluator'
        ),        
    ])