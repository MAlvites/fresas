#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32MultiArray
from std_srvs.srv import Empty
import csv
import time

MOTOR_ID_1 = 0x0042
MOTOR_ID_2 = 0x0057
MOTOR_ID_3 = 0x0001 
MOTOR_ID_4 = 0x0072

CAN_PACKET_STATUS = 9
CAN_PACKET_STATUS_2 = 14
CAN_PACKET_STATUS_3 = 15
CAN_PACKET_STATUS_4 = 16
CAN_PACKET_STATUS_5 = 27
CAN_PACKET_STATUS_6 = 28

motor_ids = {
    MOTOR_ID_1: "motor_front_left",
    MOTOR_ID_2: "motor_front_right",
    MOTOR_ID_3: "motor_back_left",
    MOTOR_ID_4: "motor_back_right",
}

command_ids = {
    CAN_PACKET_STATUS: "ERPM, Current, Duty Cycle",
    #CAN_PACKET_STATUS_2: "Ah Used, Ah Charged",
    #CAN_PACKET_STATUS_3: "Wh Used, Wh Charged",
    #CAN_PACKET_STATUS_4: "Temp Fet, Temp Motor, Current In, PID position",
    CAN_PACKET_STATUS_5: "Tachometer, Voltage In",
    #CAN_PACKET_STATUS_6: "ADC1, ADC2, ADC3, PPM",
}

class DataCollector(Node):
    def __init__(self):
        super().__init__('data_collector')

        # List to store all recorded data as dictionaries
        self.data_entries = []

        # Recording flag
        self.record_flag = False

        # Dynamic file name
        timestamp_str = time.strftime("%Y%m%d_%H%M%S")
        self.csv_filename = f'bldc_can_data_{timestamp_str}.csv'

        # Initialize the file with headers
        with open(self.csv_filename, 'w', newline='') as csvfile:
            fieldnames = ['timestamp', 'motor_id', 'rpm_setpoint', 'rpm', 'current', 'battery']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()

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

        self.record_service = self.create_service(
            Empty,
            'record_bldc_data',
            self.record_service_callback
        )

        self.get_logger().info(f"Initialized data collector, saving to {self.csv_filename}")

    def can_callback(self, msg):
        try:
            current_time = time.time()
            if self.record_flag:
                self.save_can_information(msg.data, current_time)
        except Exception as e:
            self.get_logger().error(f"Error in CAN callback: {e}")

    def rpm_setpoint_callback(self, msg):
        try:
            current_time = time.time()
            if self.record_flag:
                self.save_rpm_information(msg.data, current_time)
        except Exception as e:
            self.get_logger().error(f"Error in RPM callback: {e}")

    def save_can_information(self, can_msg, current_time):
        try:
            msg_list = can_msg.split(",")
            can_data = bytes.fromhex(msg_list[1])
            command_id = int.from_bytes(bytes.fromhex(msg_list[0])[2:3], "big", signed=True)
            motor_id = int.from_bytes(bytes.fromhex(msg_list[0])[3:4], "big", signed=True)

            if (motor_id in motor_ids) and (command_id in command_ids):
                self.get_logger().info("Recording CAN data")
                motor_name = motor_ids[motor_id]
                
                entry = {
                    'timestamp': current_time,
                    'motor_id': motor_name,
                    'rpm_setpoint': "nan",
                    'rpm': "nan",
                    'current': "nan",
                    'battery': "nan",
                }

                if command_id == CAN_PACKET_STATUS:
                    entry['current'] = int.from_bytes(can_data[4:6], "big", signed=True) / 10.0
                    entry['rpm'] = int.from_bytes(can_data[0:4], "big", signed=True)

                elif command_id == CAN_PACKET_STATUS_5:
                    entry['battery'] = int.from_bytes(can_data[4:6], "big", signed=True) / 10.0

                self.data_entries.append(entry)
                self.save_data()

        except Exception as e:
            self.get_logger().error(f"Error extracting CAN message: {e}")

    def save_rpm_information(self, rpm, current_time):
        try:
            self.get_logger().info("Recording RPM setpoint data")
            for rpm_value in rpm:
                entry = {
                    'timestamp': current_time,
                    'motor_id': "nan",
                    'rpm_setpoint': rpm_value,
                    'rpm': "nan",
                    'current': "nan",
                    'battery': "nan",
                }
                self.data_entries.append(entry)
            self.save_data()
        except Exception as e:
            self.get_logger().error(f"Error extracting RPM message: {e}")

    def save_data(self):
        if not self.data_entries:
            self.get_logger().warning("No data to save.")
            return 

        with open(self.csv_filename, 'a', newline='') as csvfile:
            fieldnames = ['timestamp', 'motor_id', 'rpm_setpoint', 'rpm', 'current', 'battery']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

            # Write data
            for entry in self.data_entries:
                writer.writerow(entry)

            self.get_logger().info(f"Data saved to {self.csv_filename}")

        # Clear the data entries after saving
        self.data_entries.clear()

    def record_service_callback(self, request, response):
        self.record_flag = not self.record_flag
        self.get_logger().info(f"Recording {'started' if self.record_flag else 'stopped'}.")
        return response
    
def main(args=None):
    rclpy.init(args=args)
    data_collector = DataCollector()

    try:
        rclpy.spin(data_collector)
    except KeyboardInterrupt:
        pass

    data_collector.get_logger().info("Shutting down, saving remaining data.")
    data_collector.save_data()  # Ensure data is saved before shutting down
    data_collector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
