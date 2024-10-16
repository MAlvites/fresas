#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, String

# Define your constants
CAN_PACKET_SET_DUTY = 0
CAN_PACKET_SET_CURRENT = 1
CAN_PACKET_SET_CURRENT_BRAKE = 2
CAN_PACKET_SET_RPM = 3
CAN_PACKET_SET_POS = 4
CAN_PACKET_SET_CURRENT_REL = 10
CAN_PACKET_SET_CURRENT_BRAKE_REL = 11
CAN_PACKET_SET_CURRENT_HANDBRAKE = 12
CAN_PACKET_SET_CURRENT_HANDBRAKE_REL = 13

CAN_PACKET_STATUS = 9
CAN_PACKET_STATUS_2 = 14
CAN_PACKET_STATUS_3 = 15
CAN_PACKET_STATUS_4 = 16
CAN_PACKET_STATUS_5 = 27
CAN_PACKET_STATUS_6 = 28

MOTOR_ID_1 = 0x0042
MOTOR_ID_2 = 0x0057
MOTOR_ID_3 = 0x0001
MOTOR_ID_4 = 0x0072

motor_ids = {
    'motor_1':MOTOR_ID_1,
    'motor_2':MOTOR_ID_2,
    'motor_3':MOTOR_ID_3,
    'motor_4':MOTOR_ID_4,
}

def check_motor_id(motor_id):
    return motor_id in motor_ids.values()

class CANHandler(Node):
    def __init__(self):
        super().__init__('can_handler')
        # Dictionary to store publishers
        self.publishers_ = dict()

        for motor in motor_ids.keys():
            self.publishers_[f'{motor}_rpm'] = self.create_publisher(Int32, f'{motor}_rpm', 10)
            self.publishers_[f'{motor}_current'] = self.create_publisher(Float32, f'{motor}_current', 10)
            self.publishers_[f'{motor}_duty'] = self.create_publisher(Float32, f'{motor}_duty', 10)

            self.publishers_[f'{motor}_amp_hours'] = self.create_publisher(Float32, f'{motor}_amp_hours', 10)
            self.publishers_[f'{motor}_amp_hours_chg'] = self.create_publisher(Float32, f'{motor}_amp_hours_chg', 10)

            self.publishers_[f'{motor}_watt_hours'] = self.create_publisher(Float32, f'{motor}_watt_hours', 10)
            self.publishers_[f'{motor}_watt_hours_chg'] = self.create_publisher(Float32, f'{motor}_watt_hours_chg', 10)

            self.publishers_[f'{motor}_temp_fet'] = self.create_publisher(Float32, f'{motor}_temp_fet', 10)
            self.publishers_[f'{motor}_temp_motor'] = self.create_publisher(Float32, f'{motor}_temp_motor', 10)
            self.publishers_[f'{motor}_current_input'] = self.create_publisher(Float32, f'{motor}_current_input', 10)
            self.publishers_[f'{motor}_pid_pos'] = self.create_publisher(Float32, f'{motor}_pid_pos', 10)

            self.publishers_[f'{motor}_tachometer'] = self.create_publisher(Float32, f'{motor}_tachometer', 10)
            self.publishers_[f'{motor}_volts_input'] = self.create_publisher(Float32, f'{motor}_volts_input', 10)

            self.publishers_[f'{motor}_adc_1'] = self.create_publisher(Float32, f'{motor}_adc_1', 10)
            self.publishers_[f'{motor}_adc_2'] = self.create_publisher(Float32, f'{motor}_adc_2', 10)
            self.publishers_[f'{motor}_adc_3'] = self.create_publisher(Float32, f'{motor}_adc_3', 10)
            self.publishers_[f'{motor}_ppm'] = self.create_publisher(Float32, f'{motor}_ppm', 10)

        self.can_subscription = self.create_subscription(
            String,
            'can_topic',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        can_msg = msg.data
        self.process_can_message(can_msg)

    def process_can_message(self, can_msg):
        if not can_msg:
            self.get_logger().error("No CAN message provided.")
            return

        msg_list = can_msg.split(",")
        if len(msg_list) != 2:
            self.get_logger().error("Invalid CAN message format.")
            return

        try:
            can_id = bytes.fromhex(msg_list[0])
            can_data = bytes.fromhex(msg_list[1])
        except ValueError as e:
            self.get_logger().error(f"Error parsing CAN message: {e}")
            return

        command_id = int.from_bytes(can_id[2:3], "big", signed=True)
        motor_id = int.from_bytes(can_id[3:4], "big", signed=True)

        if check_motor_id(motor_id):
            motor = list(motor_ids.keys())[list(motor_ids.values()).index(motor_id)]
            if command_id == CAN_PACKET_STATUS:
                rpm = int.from_bytes(can_data[0:4], "big", signed=True)
                current = int.from_bytes(can_data[4:6], "big", signed=True) / 10.0
                duty = int.from_bytes(can_data[6:8], "big", signed=True) / 1000.0
                self.publishers_[f'{motor}_rpm'].publish(Int32(data=rpm))
                self.publishers_[f'{motor}_current'].publish(Float32(data=current))
                self.publishers_[f'{motor}_duty'].publish(Float32(data=duty))

            elif command_id == CAN_PACKET_STATUS_2:
                amp_hours = int.from_bytes(can_data[0:4], "big", signed=True) / 10000.0
                amp_hours_chg = int.from_bytes(can_data[4:8], "big", signed=True) / 10000.0
                self.publishers_[f'{motor}_amp_hours'].publish(Float32(data=amp_hours))
                self.publishers_[f'{motor}_amp_hours_chg'].publish(Float32(data=amp_hours_chg))

            elif command_id == CAN_PACKET_STATUS_3:
                watt_hours = int.from_bytes(can_data[0:4], "big", signed=True) / 10000.0
                watt_hours_chg = int.from_bytes(can_data[4:8], "big", signed=True) / 10000.0
                self.publishers_[f'{motor}_watt_hours'].publish(Float32(data=watt_hours))
                self.publishers_[f'{motor}_watt_hours_chg'].publish(Float32(data=watt_hours_chg))

            elif command_id == CAN_PACKET_STATUS_4:
                temp_fet = int.from_bytes(can_data[0:2], "big", signed=True) / 10.0
                temp_motor = int.from_bytes(can_data[2:4], "big", signed=True) / 10.0
                current_input = int.from_bytes(can_data[4:6], "big", signed=True) / 10.0
                pid_pos = int.from_bytes(can_data[6:8], "big", signed=True) / 50.0
                self.publishers_[f'{motor}_temp_fet'].publish(Float32(data=temp_fet))
                self.publishers_[f'{motor}_temp_motor'].publish(Float32(data=temp_motor))
                self.publishers_[f'{motor}_current_input'].publish(Float32(data=current_input))
                self.publishers_[f'{motor}_pid_pos'].publish(Float32(data=pid_pos))

            elif command_id == CAN_PACKET_STATUS_5:
                tachometer = int.from_bytes(can_data[0:4], "big", signed=True) / 6.0
                volts_input = int.from_bytes(can_data[4:6], "big", signed=True) / 10.0
                self.publishers_[f'{motor}_tachometer'].publish(Float32(data=tachometer))
                self.publishers_[f'{motor}_volts_input'].publish(Float32(data=volts_input))

            elif command_id == CAN_PACKET_STATUS_6:
                adc_1 = int.from_bytes(can_data[0:2], "big", signed=True) / 1000.0
                adc_2 = int.from_bytes(can_data[2:4], "big", signed=True) / 1000.0
                adc_3 = int.from_bytes(can_data[4:6], "big", signed=True) / 1000.0
                ppm = int.from_bytes(can_data[6:8], "big", signed=True) / 1000.0
                self.publishers_[f'{motor}_adc_1'].publish(Float32(data=adc_1))
                self.publishers_[f'{motor}_adc_2'].publish(Float32(data=adc_2))
                self.publishers_[f'{motor}_adc_3'].publish(Float32(data=adc_3))
                self.publishers_[f'{motor}_ppm'].publish(Float32(data=ppm))
            else:
                self.get_logger().warning("Command Id not identified")
        else:
            self.get_logger().warning("Motor not identified")

def main(args=None):
    rclpy.init(args=args)
    can_handler = CANHandler()
    rclpy.spin(can_handler)
    can_handler.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()