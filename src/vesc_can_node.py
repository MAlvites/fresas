#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, String
import can
import threading

CAN_PACKET_SET_DUTY = 0
CAN_PACKET_SET_CURRENT = 1
CAN_PACKET_SET_CURRENT_BRAKE = 2
CAN_PACKET_SET_RPM = 3
CAN_PACKET_SET_POS = 4
CAN_PACKET_SET_CURRENT_REL = 10
CAN_PACKET_SET_CURRENT_BRAKE_REL = 11
CAN_PACKET_SET_CURRENT_HANDBRAKE = 12
CAN_PACKET_SET_CURRENT_HANDBRAKE_REL = 13

class CANNode(Node):
    def __init__(self):
        super().__init__('can_node')

        # Subscription to listen to RPM messages
        self.subscription_ = self.create_subscription(
            Int32MultiArray,
            'bldc_motors/rpm',
            self.listener_callback,
            10)

        # Publisher to publish received CAN messages
        self.can_publisher_ = self.create_publisher(String, 'can_received_messages', 10)

        self.vesc_id_1 = 0x0357
        self.vesc_id_2 = 0x0357
        self.vesc_id_3 = 0x0357
        self.vesc_id_4 = 0x0357

        # Initialize CAN bus
        self.bus = can.interface.Bus(bustype='slcan', channel='/dev/ttyACM0', bitrate=500000)

        # Flag to control the receive thread
        self.running = True

        # Start a thread for receiving CAN messages
        self.receive_thread = threading.Thread(target=self.receive_messages)
        self.receive_thread.start()

    def listener_callback(self, msg):
        """Callback function to process received messages."""
        rpm1 = msg.data[0].to_bytes(4, byteorder="big", signed=True)
        rpm2 = msg.data[1].to_bytes(4, byteorder="big", signed=True)
        rpm3 = msg.data[2].to_bytes(4, byteorder="big", signed=True)
        rpm4 = msg.data[3].to_bytes(4, byteorder="big", signed=True)

        self.bus.send(can.Message(
            arbitration_id=self.vesc_id_1, data=rpm1, is_extended_id=True
        ))
        self.bus.send(can.Message(
            arbitration_id=self.vesc_id_2, data=rpm2, is_extended_id=True
        ))
        self.bus.send(can.Message(
            arbitration_id=self.vesc_id_3, data=rpm3, is_extended_id=True
        ))
        self.bus.send(can.Message(
            arbitration_id=self.vesc_id_4, data=rpm4, is_extended_id=True
        ))

    def receive_messages(self):
        """Receives messages and publishes them to a ROS topic."""
        while self.running:
            try:
                message = self.bus.recv(timeout=1.0)
                if message is not None:
                    can_msg = f"ID: {message.arbitration_id}, Data: {message.data.hex()}"
                    self.get_logger().info(f"Received message: {can_msg}")
                    self.can_publisher_.publish(String(data=can_msg))
            except can.CanError as e:
                self.get_logger().error(f"CAN Error: {e}")

    def destroy_node(self):
        """Override destroy_node to stop the receive thread."""
        self.running = False
        self.receive_thread.join()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    can_node = CANNode()
    try:
        rclpy.spin(can_node)
    except KeyboardInterrupt:
        pass
    finally:
        can_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
