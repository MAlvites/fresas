import serial.tools.list_ports
from sys import platform


#Identifica SO, se conecta al puerto indicado
if platform == 'linux' or platform == 'linux2':
    def serial_ports_linux():
        ports = list(serial.tools.list_ports.comports())  
        for port_no, description, address in ports:
            if 'ACM' in description:
                return port_no
            if 'Arduino' in description:
                return port_no
            if 'Serial' in description:
                return port_no
    print(serial_ports_linux())

elif platform == 'win32':
    def serial_ports_win():
        ports = list(serial.tools.list_ports.comports())  
        for port_no, description, address in ports:
            if 'Arduino' in description:
                return port_no
            if 'CH340' in description:
                return port_no
