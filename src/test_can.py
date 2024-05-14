#!/usr/bin/env python

import can
import time

bus = can.interface.Bus(bustype='slcan', channel='COM8', bitrate=500000)
rpm=-2000
num=rpm.to_bytes(4,byteorder="big",signed="True")

def send_one():
    """Sends a single message."""
    msg = can.Message(
        arbitration_id=0x0357, data=num, is_extended_id=True
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
        time.sleep(1)
        #receive_messages()