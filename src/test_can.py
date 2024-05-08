#!/usr/bin/env python

import can
import time

bus = can.interface.Bus(bustype='slcan', channel='/dev/ttyACM0', bitrate=500000)

def send_one():
    """Sends a single message."""
    msg = can.Message(
        arbitration_id=0x0357, data=[0x00, 0x00, 0x00, 0x00], is_extended_id=True
    )
    try:
        bus.send(msg)
        print(f"Message sent on {bus.channel_info}")
    except can.CanError:
        print("Message NOT sent")

def receive_messages():
    """Receives messages."""
    message = bus.recv(0.5)
    if message is not None:
        print(f"Received message: {message}")

if __name__ == "__main__":
    while True:
        send_one()
        receive_messages()