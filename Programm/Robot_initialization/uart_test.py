#!/usr/bin/env python

import serial
import time

ser = serial.Serial("/dev/ttyS0", baudrate=115200, timeout=1)
time.sleep(2)

ser.write(b"0x04")

while True:
    if ser.in_waiting > 0:
        data = ser.readline().decode("utf-8").rstrip()
        print(f"Received data {data}")
    else:
        print("no data")

ser.close()