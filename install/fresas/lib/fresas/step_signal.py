#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Int32MultiArray
import time

class StepSignalPublisher(Node):
    def __init__(self):
        super().__init__('step_signal_publisher')
        self.rpm_publisher = self.create_publisher(Int32MultiArray, 'bldc_motors/rpm', 10)
        self.position_publisher = self.create_publisher(Int32, 'dc_motor_1/position', 10)

        self.step_value_rpm = 0
        self.step_value_position = 0

        self.timer = self.create_timer(0.1, self.publish_step_signals)

    def publish_step_signals(self):
        rpm_msg = Int32MultiArray()
        rpm_msg.data = [0, 0, self.step_value_rpm, 0]
        self.rpm_publisher.publish(rpm_msg)

        position_msg = Int32()
        position_msg.data = self.step_value_position
        self.position_publisher.publish(position_msg)

        self.get_logger().info(f"Published step signal: RPM={self.step_value_rpm}, Position={self.step_value_position}")

    def set_step_signals(self, step_value_rpm, step_value_position):
        self.step_value_rpm = step_value_rpm
        self.step_value_position = step_value_position

def main(args=None):
    rclpy.init(args=args)
    step_signal_publisher = StepSignalPublisher()

    step_signal_publisher.set_step_signals(0, 600)
    time.sleep(5)
    step_signal_publisher.set_step_signals(0, 0)
    time.sleep(5) # Example step values

    try:
        rclpy.spin(step_signal_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        step_signal_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
