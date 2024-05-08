#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import can

CAN_PACKET_SET_DUTY = 0
CAN_PACKET_SET_CURRENT = 1
CAN_PACKET_SET_CURRENT_BRAKE = 2
CAN_PACKET_SET_RPM = 3
CAN_PACKET_SET_POS = 4
CAN_PACKET_SET_CURRENT_REL = 10
CAN_PACKET_SET_CURRENT_BRAKE_REL = 11
CAN_PACKET_SET_CURRENT_HANDBRAKE = 12
CAN_PACKET_SET_CURRENT_HANDBRAKE_REL =13


class CANNode(Node):
    def __init__(self):
        super().__init__('can_node')
        self.subscription_ = self.create_subscription(
            Int32,
            'bldc_motor_1/rpm',
            self.listener_callback,
            10)
        self.subscription_
        self.can_msg = can.Message(
            arbitration_id=0x0000, data=[0x00, 0x00, 0x00, 0x00], is_extended_id=True
        )
        self.vesc_id=0x0357
        # Initialize CAN bus
        self.bus = can.interface.Bus(bustype='slcan', channel='/dev/ttyACM0', bitrate=500000)

    def listener_callback(self, msg):
        """Callback function to process received messages."""
        rpm= msg.data
        self.can_msg = can.Message(
            arbitration_id=self.vesc_id, data=[0x00, 0x00, (rpm>>8)&0xff, rpm&0xff], is_extended_id=True
        )
        self.bus.send(self.can_msg)
        try:
            
            self.get_logger().info(f"Message sent")
        except can.CanError:
            self.get_logger().error("Message NOT sent")

    def receive_messages(self):
        """Receives messages."""
        while rclpy.ok():
            message = self.bus.recv()
            if message is not None:
                self.get_logger().info(f"Received message: {message}")
                # Aquí puedes publicar el mensaje CAN recibido como un mensaje ROS si lo deseas

def main(args=None):
    rclpy.init(args=args)
    can_node = CANNode()
    rclpy.spin(can_node)
    can_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()