from launch import LaunchDescription
from launch_ros.actions import Node

import numpy as np
import random
import tensorflow as tf


# set np.random.seed(0) to make the results reproducible
np.random.seed(0)
# set the tensorflow random seed
tf.random.set_seed(0)
# set the python random seed
random.seed(0)

import sys


def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='perplexity',
        #     executable='differentiator',
        #     name='differentiator'
        # ),
        # Node(
        #     package='perplexity',
        #     executable='receiver',
        #     name='receiver'
        # ),
        # Node(
        #     package='perplexity',
        #     executable='bufferer',
        #     name='bufferer'
        # ),
        Node(
            package='perplexity',
            executable='inferencer',
            name='inferencer',
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