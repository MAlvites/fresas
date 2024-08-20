#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32MultiArray
from std_srvs.srv import Empty
import csv
import time

class DataCollector(Node):
    def __init__(self):
        super().__init__('data_collector')
        
        # Lists to store timestamps and data
        self.time_stamps_rpm = []
        self.rpm_setpoint = []
        
        self.time_stamps_can = []
        self.can_data = []

        # Subscriptions
        self.subscription_can = self.create_subscription(
            String,
            'can_topic',
            self.can_callback,
            10)

        self.subscription_rpm = self.create_subscription(
            Int32MultiArray,
            'bldc_motors/rpm',
            self.rpm_setpoint_callback,
            10)

        self.record_flag = False

        self.record_service = self.create_service(
            Empty,
            'record_bldc_data',
            self.record_service_callback
        )

        # Initialize the file with headers
        with open('can_data.csv', 'w', newline='') as csvfile:
            fieldnames = ['timestamp', 'rpm_setpoint', 'rpm', 'current', 'battery']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()

    def can_callback(self, msg):
        try:
            if self.record_flag:
                self.get_logger().info("Recording")
                current_time = time.time()
                rpm_can = extract_rpm_can(msg.data)
                current_can = extract_current_can(msg.data)
                battery_can = extract_battery_can(msg.data)
                if rpm_can != "nan" or current_can != "nan" or battery_can != "nan":
                    self.time_stamps_can.append(current_time)
                    self.can_data.append((rpm_can, current_can, battery_can))
                    self.get_logger().info(f"Received CAN message at {current_time}: RPM={rpm_can}, Current={current_can}, Battery={battery_can}")
                self.save_data()
        except Exception as e:
            self.get_logger().error(f"Error in CAN callback: {e}")

    def rpm_setpoint_callback(self, msg):
        try:
            if self.record_flag:
                self.get_logger().info("Recording")
                current_time = time.time()
                self.time_stamps_rpm.append(current_time)
                self.rpm_setpoint.append(msg.data[2])
                #self.get_logger().info(f"Received RPM message at {current_time}: {msg.data}")
                self.save_data()
        except Exception as e:
            self.get_logger().error(f"Error in RPM callback: {e}")

    def save_data(self):
        if not self.time_stamps_can and not self.time_stamps_rpm:
            self.get_logger().warning("No data to save.")
            return 

        with open('can_data.csv', 'a', newline='') as csvfile:
            fieldnames = ['timestamp', 'rpm_setpoint', 'rpm', 'current', 'battery']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

            # Write CAN data (including RPM, Current, Battery)
            for timestamp, can_data in zip(self.time_stamps_can, self.can_data):
                rpm_can, current_can, battery_can = can_data
                writer.writerow({'timestamp': timestamp, 'rpm_setpoint': '', 'rpm': rpm_can, 'current': current_can, 'battery': battery_can})

            # Write RPM data
            for timestamp, rpm_setpoint in zip(self.time_stamps_rpm, self.rpm_setpoint):
                writer.writerow({'timestamp': timestamp, 'rpm_setpoint': rpm_setpoint, 'rpm': '', 'current': '', 'battery': ''})

        self.get_logger().info("Data saved to can_data.csv")

        # Clear lists after saving
        self.time_stamps_can.clear()
        self.can_data.clear()
        self.time_stamps_rpm.clear()
        self.rpm_setpoint.clear()

    def record_service_callback(self, request, response):
        self.record_flag = not self.record_flag
        return response

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

    return "nan"

def extract_current_can(can_msg):
    try:
        msg_list = can_msg.split(",")
        can_data = bytes.fromhex(msg_list[1])
        command_id = int.from_bytes(bytes.fromhex(msg_list[0])[2:3], "big", signed=True)
        motor_id = int.from_bytes(bytes.fromhex(msg_list[0])[3:4], "big", signed=True)

        if motor_id == 0x0001 and command_id == 9:
            current = int.from_bytes(can_data[4:6], "big", signed=True)/10.0
            return current
    except Exception as e:
        print(f"Error extracting RPM from CAN message: {e}")

    return "nan"

def extract_battery_can(can_msg):
    try:
        msg_list = can_msg.split(",")
        can_data = bytes.fromhex(msg_list[1])
        command_id = int.from_bytes(bytes.fromhex(msg_list[0])[2:3], "big", signed=True)
        motor_id = int.from_bytes(bytes.fromhex(msg_list[0])[3:4], "big", signed=True)

        if motor_id == 0x0001 and command_id == 27:
            battery = int.from_bytes(can_data[4:6], "big", signed=True)/10.0
            return battery
    except Exception as e:
        print(f"Error extracting RPM from CAN message: {e}")

    return "nan"

if __name__ == '__main__':
    main()