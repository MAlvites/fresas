#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

class RpmPublisher(Node):
    def __init__(self):
        super().__init__('rpm_publisher')
        self.publisher_ = self.create_publisher(Int32MultiArray, 'bldc_motors/rpm', 10)
        self.timer_publish = self.create_timer(0.5, self.publish_rpm)
        self.timer_update = self.create_timer(3.0, self.update_rpm)
        self.rpm_data = [[0, 0, 0, 0], [1000, 1000, 1000, 1000], [2000, 2000, 2000, 2000], [3000, 3000, 3000, 3000],[2000, 2000, 2000, 2000],[1000, 1000, 1000, 1000],[0, 0, 0, 0]]
        self.index = 0

    def publish_rpm(self):
        msg = Int32MultiArray()
        msg.data = self.rpm_data[self.index]
        self.publisher_.publish(msg)

    def update_rpm(self):
        self.index = (self.index + 1) % len(self.rpm_data)

def main(args=None):
    rclpy.init(args=args)
    rpm_publisher = RpmPublisher()
    try:
        rclpy.spin(rpm_publisher)
    except KeyboardInterrupt:
        pass
    rpm_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()