from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='perplexity',
            executable='differentiator',
            name='differentiator'
        ),
        Node(
            package='perplexity',
            executable='receiver',
            name='receiver'
        ),
        Node(
            package='perplexity',
            executable='inferencer',
            name='inferencer'
        ),
        Node(
            package='perplexity',
            executable='trainer',
            name='trainer'
        ),
        Node(
            package='perplexity',
            executable='evaluator',
            name='evaluator'
        ),        
    ])