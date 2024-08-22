#!/usr/bin/env python

import can
import struct
import time

# Configuration of the CAN interface (adjust as necessary)
can_interface = 'can0'  # The name of your CAN interface, e.g., 'can0' on Linux
bitrate = 1000000  # The bitrate of your CAN bus

# Function to send a CAN message
def send_can_message(bus, arbitration_id, data):
    message = can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=False)
    try:
        bus.send(message)
        print(f"Message sent on {bus.channel_info}")
    except can.CanError:    
        print("Message NOT sent")

# Function to set the absolute position of the RMD motor
def set_absolute_position(bus, position):
    # Convert the position to bytes (assuming a 32-bit integer position value)
    position_bytes = struct.pack('<i', position)  # Little-endian format

    # Create the CAN data message
    data = [0xA2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    

    # Send the message with the appropriate CAN ID (adjust the ID as necessary)
    arbitration_id = 0x141  # The CAN ID for the RMD motor
    send_can_message(bus, arbitration_id, data)

# Initialize the CAN bus
bus = can.interface.Bus(channel=can_interface, bustype='socketcan', bitrate=bitrate)

# Example usage: Set the motor to an absolute position of 10000 (adjust as necessary)
desired_position = 1000
set_absolute_position(bus, desired_position)

# Allow some time for the message to be sent and processed
time.sleep(0.1)

# Close the CAN bus
bus.shutdown()