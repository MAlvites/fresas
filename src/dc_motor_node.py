#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_srvs.srv import Empty
import serial
import time

class DCMotorNode(Node):
    def __init__(self):
        super().__init__('dc_motor_node')
        
        # Declare parameters with default values
        self.declare_parameter('serial_port', '/dev/ttyACM2')
        
        # Get the parameter value
        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        
        # Initialize serial port
        self.init_serial(serial_port)

        # Initialize subscriptions and services
        self.position_subscription = self.create_subscription(
            Int32,
            'position',
            self.position_callback,
            10
        )
        
        self.rel_position_subscription = self.create_subscription(
            Int32,
            'rel_position',
            self.rel_position_callback,
            10
        )
        
        self.zero_set_service = self.create_service(
            Empty,
            'zero_set',
            self.zero_set_callback
        )
        
        self.clear_errors_service = self.create_service(
            Empty,
            'clear_errors',
            self.clear_errors_callback
        )

    def init_serial(self, serial_port):
        try:
            self.serial_port_instance = serial.Serial(
                port=serial_port,
                baudrate=115200,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
            )
            self.get_logger().info(f'UART initialized for DC motor on port: {serial_port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Error initializing UART: {e}')
            raise e

    def position_callback(self, msg):
        try:
            position = msg.data
            command = f"N24 p{position} v100/n O G3/n"
            self.serial_port_instance.write(command.encode())
        except serial.SerialException as e:
            self.get_logger().error(f'Error communicating with motor: {e}')

    def rel_position_callback(self, msg):
        try:
            rel_position = msg.data
            command = f"N24 P{rel_position} v100/n"
            self.serial_port_instance.write(command.encode())
        except serial.SerialException as e:
            self.get_logger().error(f'Error communicating with motor: {e}')

    def zero_set_callback(self, request, response):
        try:
            self.serial_port_instance.write("N24 O r/n".encode())
            self.get_logger().info('Zero set')
            return response
        except serial.SerialException as e:
            self.get_logger().error(f'Error communicating with motor: {e}')
            return response

    def clear_errors_callback(self, request, response):
        try:
            time.sleep(1)
            self.serial_port_instance.write("N24 O C/n".encode())
            self.get_logger().info('Errors cleared')
            return response
        except serial.SerialException as e:
            self.get_logger().error(f'Error communicating with motor: {e}')
            return response

def main(args=None):
    rclpy.init(args=args)
    node = DCMotorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
