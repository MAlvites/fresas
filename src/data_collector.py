#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32MultiArray
import csv
import time

class DataCollector(Node):
    def __init__(self):
        super().__init__('data_collector')
        
        # Lists to store timestamps and data
        self.time_stamps_can = []
        self.can_data = []

        self.time_stamps_rpm = []
        self.rpm_data = []

        # Subscriptions
        self.subscription_can = self.create_subscription(
            String,
            'can_topic',
            self.can_callback,
            10)

        self.subscription_rpm = self.create_subscription(
            Int32MultiArray,
            'bldc_motors/rpm',
            self.rpm_callback,
            10)

        # Timer to check for data and save periodically
        self.create_timer(5.0, self.check_and_save)

        # Initialize the file with headers
        with open('can_rpm_data.csv', 'w', newline='') as csvfile:
            fieldnames = ['timestamp', 'can_data', 'rpm_data']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()

    def can_callback(self, msg):
        try:
            current_time = time.time()
            rpm_can = extract_rpm_can(msg.data)
            if rpm_can is not None:
                self.time_stamps_can.append(current_time)
                self.can_data.append(rpm_can)
                self.get_logger().info(f"Received CAN message at {current_time}: {msg.data}")
        except Exception as e:
            self.get_logger().error(f"Error in CAN callback: {e}")

    def rpm_callback(self, msg):
        try:
            current_time = time.time()
            self.time_stamps_rpm.append(current_time)
            self.rpm_data.append(msg.data[2])
            self.get_logger().info(f"Received RPM message at {current_time}: {msg.data}")
        except Exception as e:
            self.get_logger().error(f"Error in RPM callback: {e}")

    def check_and_save(self):
        if self.time_stamps_can or self.time_stamps_rpm:
            self.save_data()

    def save_data(self):
        if not self.time_stamps_can and not self.time_stamps_rpm:
            self.get_logger().warning("No data to save.")
            return

        with open('can_rpm_data.csv', 'a', newline='') as csvfile:
            fieldnames = ['timestamp', 'can_data', 'rpm_data']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

            # Write CAN data
            for timestamp, can_data in zip(self.time_stamps_can, self.can_data):
                writer.writerow({'timestamp': timestamp, 'can_data': can_data, 'rpm_data': ''})

            # Write RPM data
            for timestamp, rpm_data in zip(self.time_stamps_rpm, self.rpm_data):
                writer.writerow({'timestamp': timestamp, 'can_data': '', 'rpm_data': rpm_data})

        self.get_logger().info("Data saved to can_rpm_data.csv")

        # Clear lists after saving
        self.time_stamps_can.clear()
        self.can_data.clear()
        self.time_stamps_rpm.clear()
        self.rpm_data.clear()

def main(args=None):
    rclpy.init(args=args)
    data_collector = DataCollector()

    try:
        rclpy.spin(data_collector)
    except KeyboardInterrupt:
        pass
    
    data_collector.save_data()  # Ensure data is saved before shutting down
    data_collector.destroy_node()
    rclpy.shutdown()

def extract_rpm_can(can_msg):
    try:
        msg_list = can_msg.split(",")
        can_data = bytes.fromhex(msg_list[1])
        command_id = int.from_bytes(bytes.fromhex(msg_list[0])[2:3], "big", signed=True)
        motor_id = int.from_bytes(bytes.fromhex(msg_list[0])[3:4], "big", signed=True)

        if motor_id == 0x0001 and command_id == 9:
            rpm = int.from_bytes(can_data[0:4], "big", signed=True)
            return rpm
    except Exception as e:
        print(f"Error extracting RPM from CAN message: {e}")

    return None

if __name__ == '__main__':
    main()