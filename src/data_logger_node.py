#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
import csv

from sensor_msgs.msg import JointState
from std_srvs.srv import Empty
from dc_motor_interfaces.msg import MotorStateStamped

class MotorStateLogger(Node):

    def __init__(self):
        super().__init__('motor_state_logger')

        # Declare and get the motor_id parameter as a string
        self.declare_parameter('motor_id', 'dc_motor_1')
        self.motor_id = self.get_parameter('motor_id').get_parameter_value().string_value

        # Create a CSV file with the motor_id in the filename
        self.nombre_archivo = f'datos_{self.motor_id}.csv'
        self.record_flag = False

        # Write the CSV header
        with open(self.nombre_archivo, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Timestamp', 'Position (rad)', 'Velocity (rad/s)', 'Current (A)'])
        
        # Subscribe to the dc_motor_state topic
        self.pose_subscriber_ = self.create_subscription(MotorStateStamped, 'dc_motor_state', self.obtain_data, 1)
        
        # Create a service for toggling recording
        self.record_service = self.create_service(
            Empty,
            'record_dc_data',
            self.record_service_callback
        )

    def obtain_data(self, msg = MotorStateStamped):
        if self.record_flag:
            self.get_logger().info("Recording")
            timestamp = time.strftime('%Y-%m-%d %H:%M:%S')
            position = msg.state.position
            velocity = msg.state.velocity
            current = msg.state.current
        
            try:
                with open(self.nombre_archivo, mode='a', newline='') as file:
                    writer = csv.writer(file)
                    writer.writerow([timestamp, position, velocity, current])

            except KeyboardInterrupt:
                print("Detenido.")

    def record_service_callback(self, request, response):
        # Toggle the recording flag
        self.record_flag = not self.record_flag
        return response

def main(args=None):
    rclpy.init(args=args)
    
    # Create and spin the node
    motor_state_logger = MotorStateLogger()
    rclpy.spin(motor_state_logger)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
