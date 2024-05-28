#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='fresas',
            executable='can_node',
            name='can_node',
            output='screen'
        ),
        Node(
            package='fresas',
            executable='can_handler',
            name='can_handler',
            output='screen'
        )
    ])
