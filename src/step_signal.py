#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Int32MultiArray

class StepSignalPublisher(Node):
    def __init__(self):
        super().__init__('step_signal_publisher')
        self.rpm_publisher = self.create_publisher(Int32MultiArray, 'bldc_motors/rpm', 10)
        self.position_publisher = self.create_publisher(Int32, 'dc_motor_1/position', 10)

        self.step_value_rpm = 0
        self.step_value_position = 0

        # Start with the first step signal and move to the next after 5 seconds
        self.step_index = 0
        self.step_signals = [
            (2000, 400),  # Signal 1
            (1500, 600),  # Signal 2
            (1000, 800)   # Signal 3
        ]

        # Publishing step signals every 0.1 seconds
        self.timer = self.create_timer(0.1, self.publish_step_signals)

        # Timer to change signals every 5 seconds
        self.step_change_timer = self.create_timer(5.0, self.change_step_signal)

    def publish_step_signals(self):
        rpm_msg = Int32MultiArray()
        rpm_msg.data = [0, 0, self.step_value_rpm, 0]
        self.rpm_publisher.publish(rpm_msg)

        position_msg = Int32()
        position_msg.data = self.step_value_position
        self.position_publisher.publish(position_msg)

        self.get_logger().info(f"Published step signal: RPM={self.step_value_rpm}, Position={self.step_value_position}")

    def change_step_signal(self):
        if self.step_index < len(self.step_signals):
            self.set_step_signals(*self.step_signals[self.step_index])
            self.get_logger().info(f"Set step signal {self.step_index + 1}: RPM={self.step_signals[self.step_index][0]}, Position={self.step_signals[self.step_index][1]}")
            self.step_index += 1
        else:
            self.stop_publishing()

    def set_step_signals(self, step_value_rpm, step_value_position):
        self.step_value_rpm = step_value_rpm
        self.step_value_position = step_value_position

    def stop_publishing(self):
        # Stop both the publishing and step change timers
        self.timer.cancel()
        self.step_change_timer.cancel()
        self.get_logger().info("Stopped publishing step signals.")

def main(args=None):
    rclpy.init(args=args)
    step_signal_publisher = StepSignalPublisher()

    try:
        rclpy.spin(step_signal_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        step_signal_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
