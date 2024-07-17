#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='fresas',
            executable='dc_motor_node',
            name='dc_motor_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyACM1'
            }]
        ),
        Node(
            package='fresas',
            executable='vesc_can_node',
            name='vesc_can_node',
            output='screen',
            parameters=[{
                'can_channel': '/dev/ttyACM2'
            }]
        ),
        Node(
            package='fresas',
            executable='vesc_can_handler',
            name='vesc_can_handler',
            output='screen',
        ),
        Node(
            package='fresas',
            executable='gamepad_control_node',
            name='gamepad_control_node',
            output='screen',
        )
    ])

