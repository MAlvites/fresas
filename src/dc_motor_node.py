#!/usr/bin/python3
import time
import serial
import argparse

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_srvs.srv import Empty

class DCMotorNode(Node):
    def __init__(self, serial_port):
        super().__init__('dc_motor_1')
        self.serial_port = None
        self.serial_port_name = serial_port
        self.init_serial()

        self.position_subscription = self.create_subscription(
            Int32,
            'dc_motor_1/position',
            self.position_callback,
            10)
        
        self.rel_position_subscription = self.create_subscription(
            Int32,
            'dc_motor_1/rel_position',
            self.rel_position_callback,
            10)
        
        self.zero_set_service = self.create_service(
            Empty,
            'dc_motor_1/zero_set',
            self.zero_set_callback)

    def init_serial(self):
        try:
            self.serial_port = serial.Serial(
                port=self.serial_port_name,
                baudrate=115200,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
            )
            time.sleep(1)  # Esperar a que el puerto se inicialice
            self.get_logger().info('UART initialized for DC motor on port: %s' % self.serial_port_name)
        except serial.SerialException as e:
            self.get_logger().error('Error initializing UART: %s' % str(e))
            raise e

    def position_callback(self, msg):
        try:
            position = msg.data
            command = "N24 p{} v100/n".format(position)
            self.serial_port.write(command.encode())
        except serial.SerialException as e:
            self.get_logger().error('Error communicating with motor: %s' % str(e))

    def rel_position_callback(self, msg):
        try:
            rel_position = msg.data
            command = "N24 P{} v100/n".format(rel_position)
            self.serial_port.write(command.encode())
        except serial.SerialException as e:
            self.get_logger().error('Error communicating with motor: %s' % str(e))

    def zero_set_callback(self, request, response):
        try:
            # Enviar comando al motor para redefinir cero
            self.serial_port.write("N24 O r".encode())
            # Publicar respuesta
            self.get_logger().info('Zero set')
            return response
        except serial.SerialException as e:
            self.get_logger().error('Error communicating with motor: %s' % str(e))
            return response


def main():
    parser = argparse.ArgumentParser(description='DC Motor Node')
    parser.add_argument('--serial-port', dest='serial_port', type=str, default='/dev/ttyACM0', help='Serial port name (default: /dev/ttyACM0)')
    args = parser.parse_args()

    rclpy.init(args=None)
    try:
        node = DCMotorNode(args.serial_port)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()