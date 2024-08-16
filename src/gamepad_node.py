#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32, Int32MultiArray
import serial
from std_srvs.srv import Empty

class GamepadControlNode(Node):
    def __init__(self):
        super().__init__('gamepad_control_node')
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10)
        self.publisher_bldc_rpm = self.create_publisher(Int32MultiArray, 'bldc_motors/rpm', 10)
        self.publisher_dc_motor_position = self.create_publisher(Int32, 'dc_motor_1/position', 10)

        self.var1 = 0
        self.var2 = 0

        self.prev_start_button_state = False
        self.prev_motors_state = False

        self.prev_select_button_state = False

        # Initialize serial port
        self.serial_port = serial.Serial('/dev/ttyACM0', 9600, timeout=1)  # Adjust port and baud rate as needed

        # Create service clients for each motor with the correct namespace
        self.clear_errors_clients = [
            self.create_client(Empty, '/dc_motor_1/clear_errors')#,
            #self.create_client(Empty, '/dc_motor_2/clear_errors'),
            #self.create_client(Empty, '/dc_motor_3/clear_errors'),
            #self.create_client(Empty, '/dc_motor_4/clear_errors')
        ]
        self.record_client = self.create_client(Empty,'/record_data')

        # Wait for all services to become available
        for i, client in enumerate(self.clear_errors_clients):
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for clear_errors service {i+1} to become available...')

    def scale_value(self, input_value, input_min, input_max, output_min, output_max):
        # Scale the input_value from the range [input_min, input_max] to [output_min, output_max]
        return output_min + (float(input_value - input_min) / float(input_max - input_min)) * (output_max - output_min)

    def joy_callback(self, msg):
        R1_pressed = msg.buttons[5]  # R1 button
        L3_axis = msg.axes[1]
        R3_left = msg.axes[2] < -0.5  # Joystick derecho izquierda
        R3_right = msg.axes[2] > 0.5  # Joystick derecho derecha

        start_button_pressed = msg.buttons[9]  # Start button
        select_button_pressed = msg.buttons[8] #Select button

        if R1_pressed:
            if L3_axis > 0.05:
                self.var1 = self.scale_value(L3_axis, 0.05, 1, 800, 3000)
            elif L3_axis < -0.05:
                self.var1 = self.scale_value(L3_axis, -1, -0.05, -3000, -800)
            else:
                self.var1 = 0

            if R3_left:
                self.var2 -= 5
            elif R3_right:
                self.var2 += 5

            if self.var2 > 675:
                self.var2 = 675
            elif self.var2 < -675:
                self.var2 = -675

            self.publisher_bldc_rpm.publish(Int32MultiArray(data=[0, 0, int(self.var1), 0]))
        self.publisher_dc_motor_position.publish(Int32(data=int(self.var2)))

        # Handle Start button press for serial communication
        if start_button_pressed:
            if not self.prev_start_button_state:
                if not self.prev_motors_state:
                    print("A")
                    self.send_character('A')
                    self.prev_motors_state = True
                else:
                    print("B")
                    self.send_character('B')
                    self.prev_motors_state = False
                self.prev_start_button_state = True
        else:
            self.prev_start_button_state = False

        #Handle Select button press for recording of data
        if select_button_pressed:
            if not self.prev_select_button_state:
                self.call_record_service()
                self.prev_select_button_state = True
        else:
            self.prev_select_button_state = False


    def send_character(self, character):
        if self.serial_port.is_open:
            self.serial_port.write(character.encode())  # Send character through serial port

            # Check if the character is 'A' and call the clear_errors service for each motor
            if character == 'A':
                self.call_clear_errors_services()

    def call_clear_errors_services(self):
        # Call clear_errors service for each motor
        for i, client in enumerate(self.clear_errors_clients):
            request = Empty.Request()
            self.get_logger().info(f'Calling clear_errors service for motor {i+1}...')
            client.call_async(request)

    def call_record_service(self):
        # Call clear_errors service for each motor    
        request = Empty.Request()
        self.get_logger().info(f'Calling record service')
        self.record_client.call_async(request)
    
    def __del__(self):
        # Ensure the serial port is closed when the node is destroyed
        if self.serial_port.is_open:
            self.serial_port.close()

def main(args=None):
    rclpy.init(args=args)
    node = GamepadControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
