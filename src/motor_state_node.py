#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import time
import re
import time
import serial
import argparse

from sensor_msgs.msg import JointState
from rclpy.node import Node
from std_msgs.msg import Float32
from dc_motor_interfaces.msg import MotorState
from dc_motor_interfaces.msg import MotorStateStamped

class MotorStateReader(Node):

    def __init__(self, serial_port):
        super().__init__('motor_state') # Inicializamos el nodo
        self.serial_port = None
        self.serial_port_name = serial_port
        self.motor_pub = self.create_publisher(MotorStateStamped, 'dc_motor_state', 1)
        self.init_serial()

    def init_serial(self):
        try:
            self.serial_port = serial.Serial(
                port=self.serial_port_name,
                baudrate=115200,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
            )
            time.sleep(1)  # Esperar a que el puerto se inicialice
            self.get_logger().info('UART initialized for DC motor on port: %s' % self.serial_port_name)
            self.get_logger().info('Leyendo datos del motor ...')
            self.correct_position()
        except serial.SerialException as e:
            self.get_logger().error('Error initializing UART: %s' % str(e))
            raise e
    

    def correct_position(self):
        while True:
            try:
                resultado = self.serial_port.read_until(b'H0')
                resultado = resultado.decode('utf-8')
                separador = '/n\n\r'

                division = resultado.split("\n",2)
                resultado = str(division[2])

                patron_posicion = r"p(.+)v"
                p_value = re.search(patron_posicion, resultado)

                patron_velocidad = r"v(.+)I"
                v_value = re.search(patron_velocidad, resultado)

                patron_corriente = r"I(.+)E0"
                i_value = re.search(patron_corriente, resultado)

                self.position_real = p_value.group(1)
                self.velocidad_real = v_value.group(1)
                self.corriente_real = float(i_value.group(1))/10000
                
                """
                print("--------------------------------------------")
                print(f'time: {self.get_clock().now().to_msg()}')
                print(f'preal: {self.position_real}')
                print(f'velocity: {self.velocidad_real}')
                print(f'current: {self.corriente_real}')
                """

                msg_state = MotorState() 
                msg_state.position = float(self.position_real)
                msg_state.velocity = float(self.velocidad_real)
                msg_state.current = float(self.corriente_real)


                msg_stamped = MotorStateStamped()
                msg_stamped.header.stamp = self.get_clock().now().to_msg()
                msg_stamped.header.frame_id = "base_link"
                msg_stamped.state = msg_state

                self.motor_pub.publish(msg_stamped)

            except IndexError:
                print("errrorrrr")



def main(args=None):
    parser = argparse.ArgumentParser(description='DC Motor Node')
    parser.add_argument('--serial-port', dest='serial_port', type=str, default='/dev/ttyACM2', help='Serial port name (default: /dev/ttyACM0)')
    args = parser.parse_args()

    rclpy.init(args=None)
    try: # Intentamos mantener el nodo en ejecución
        node = MotorStateReader(args.serial_port)
        rclpy.spin(node) 
    except KeyboardInterrupt: # Si se presiona Ctrl+C, se cierra el nodo
        pass
    rclpy.shutdown() # Cerramos las comunicaciones de ROS

if __name__ == '__main__':
    main()