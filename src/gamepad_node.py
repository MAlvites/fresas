#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32, Int32MultiArray

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

    def scale_value(self, input_value, input_min, input_max, output_min, output_max):
        # Scale the input_value from the range [input_min, input_max] to [output_min, output_max]
        return output_min + (float(input_value - input_min) / float(input_max - input_min)) * (output_max - output_min)

    def joy_callback(self, msg):
        R1_pressed = msg.buttons[5]  # R1 button
        #L3_up = msg.axes[1] < -0.5  # Joystick izquierdo arrib
        #L3_down = msg.axes[1] > 0.5  # Joystick izquierdo abajo
        L3_axis = msg.axes[1]
        R3_left = msg.axes[2] < -0.5  # Joystick derecho izquierda
        R3_right = msg.axes[2] > 0.5  # Joystick derecho derecha

        if R1_pressed:

            # Escalar el valor de L3_axis (rango [-1, 1]) a [800, 1500]
            #if L3_axis < -0.02 or L3_axis > 0.02
            #if L3_up:
            #    self.var1 += 100
            #elif L3_down:
            #    self.var1 -= 100

            if L3_axis > 0.05:
                self.var1 = self.scale_value(L3_axis, 0.05, 1, 800, 3000)
            elif L3_axis < -0.05:
                self.var1 = self.scale_value(L3_axis, -1, -0.05, -3000, -800)
            else:
                self.var1 = 0


            if R3_left:
                self.var2 -= 10
            elif R3_right:
                self.var2 += 10

            if self.var2 > 675:
                self.var2 = 675
            elif self.var2 < -675:
                self.var2 = -675

            self.publisher_bldc_rpm.publish(Int32MultiArray(data=[0,0,int(self.var1),0]))
            self.publisher_dc_motor_position.publish(Int32(data=int(self.var2)))

def main(args=None):
    rclpy.init(args=args)
    node = GamepadControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()