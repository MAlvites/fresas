#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),
        Node(
            package='fresas',
            executable='gamepad_node.py',
            name='gamepad_control_node',
            output='screen'
        ),
        Node(
            package='fresas',
            executable='vesc_can_node.py',
            name='vesc_can_node',
            output='screen'
        ),
        Node(
            package='fresas',
            executable='vesc_can_handler.py',
            name='vesc_can_handler',
            output='screen'
        ),
        Node(
            package='fresas',
            executable='dc_motor_node.py',
            namespace='dc_motor_1',
            name='dc_motor_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyACM2'
            }]
        ),
        # Node(
        #     package='fresas',
        #     executable='dc_motor_node.py',
        #     namespace='dc_motor_2',
        #     name='dc_motor_node',
        #     output='screen',
        #     parameters=[{
        #         'serial_port': '/dev/ttyACM3'
        #     }]
        # ),
        # Node(
        #     package='fresas',
        #     executable='dc_motor_node.py',
        #     namespace='dc_motor_3',
        #     name='dc_motor_node',
        #     output='screen',
        #     parameters=[{
        #         'serial_port': '/dev/ttyACM4'
        #     }]
        # ),
        # Node(
        #     package='fresas',
        #     executable='dc_motor_node.py',
        #     namespace='dc_motor_4',
        #     name='dc_motor_node',
        #     output='screen',
        #     parameters=[{
        #         'serial_port': '/dev/ttyACM5'
        #     }]
        # ),
         Node(
             package='fresas',
             executable='motor_state_node.py',
             name='motor_state_node',
             namespace='dc_motor_1',
             output='screen',
             parameters=[{
                 'serial_port': '/dev/ttyACM2'
             }]
         ),
        # Node(
        #     package='fresas',
        #     executable='motor_state_node.py',
        #     name='motor_state_node',
        #     namespace='dc_motor_2',
        #     output='screen',
        #     parameters=[{
        #         'serial_port': '/dev/ttyACM3'
        #     }]
        # ),
        # Node(
        #     package='fresas',
        #     executable='motor_state_node.py',
        #     name='motor_state_node',
        #     namespace='dc_motor_3',
        #     output='screen',
        #     parameters=[{
        #         'serial_port': '/dev/ttyACM4'
        #     }]
        # ),
        # Node(
        #     package='fresas',
        #     executable='motor_state_node.py',
        #     name='motor_state_node',
        #     namespace='dc_motor_4',
        #     output='screen',
        #     parameters=[{
        #         'serial_port': '/dev/ttyACM5'
        #     }]
        # ),
        Node(
            package='fresas',
            executable='data_logger_node.py',
            name='data_logger_node',
            namespace='dc_motor_1',
            output='screen',
            parameters=[{
                 'motor_id': 'dc_motor_1'
             }]
        ),
        # Node(
        #     package='fresas',
        #     executable='data_logger_node.py',
        #     name='data_logger_node',
        #     namespace='dc_motor_2',
        #     output='screen',
        #     parameters=[{
        #          'motor_id': 'dc_motor_2'
        #      }]
        # ),
        # Node(
        #     package='fresas',
        #     executable='data_logger_node.py',
        #     name='data_logger_node',
        #     namespace='dc_motor_3',
        #     output='screen',
        #     parameters=[{
        #          'motor_id': 'dc_motor_3'
        #      }]
        # ),
        # Node(
        #     package='fresas',
        #     executable='data_logger_node.py',
        #     name='data_logger_node',
        #     namespace='dc_motor_4',
        #     output='screen',
        #     parameters=[{
        #          'motor_id': 'dc_motor_4'
        #      }]
        # ),
        Node(
            package='fresas',
            executable='data_collector.py',
            name='data_collector_node',
            output='screen',
        )
    ])