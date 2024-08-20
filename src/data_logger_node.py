#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import time
import re
import time
import serial
import argparse
import csv

from sensor_msgs.msg import JointState
from rclpy.node import Node
from std_msgs.msg import Float32
from std_srvs.srv import Empty
from dc_motor_interfaces.msg import MotorState
from dc_motor_interfaces.msg import MotorStateStamped

class MotorStateLogger(Node):

    def __init__(self):
        super().__init__('motor_state')
        self.nombre_archivo = 'datos_motor.csv'
        with open(self.nombre_archivo, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Timestamp', 'Position (rad)', 'Velocity (rad/s)',  'Current (A)'])
        self.pose_subscriber_ = self.create_subscription(MotorStateStamped, 'dc_motor_state', self.obtain_data,1)
        self.record_flag = False
        self.record_service = self.create_service(
            Empty,
            'record_dc_data',
            self.record_service_callback
        )


    def obtain_data(self, msg = MotorStateStamped):
        if self.record_flag:
            self.get_logger().info("Recording")
            self.time = timestamp = time.strftime('%Y-%m-%d %H:%M:%S')
            self.sec= msg.header.stamp.sec
            self.position = msg.state.position
            self.velocity = msg.state.velocity
            self.current = msg.state.current
        
            try:
                with open(self.nombre_archivo, mode='a', newline='') as file:
                    writer = csv.writer(file)
                    writer.writerow([timestamp, self.position, self.velocity, self.current])

            except KeyboardInterrupt:
                print("Detenido.")

    def record_service_callback(self, request, response):
        self.record_flag = not self.record_flag
        return response

def main(args=None):
    rclpy.init(args=args)
    prueba = MotorStateLogger()
    rclpy.spin(prueba)
    rclpy.shutdown()

if __name__ == '__main__':
    main()