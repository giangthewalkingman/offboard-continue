#!/usr/bin/python
# -*- coding:utf-8 -*-
import smbus
import time

address = 0x20

bus = smbus.SMBus(1)
while True:
    bus.write_byte_data(address,0xA5,0x5A)
    time.sleep(0.5)
    bus.write_byte_data(address,0xA5,0x5A)
time.sleep(0.5)