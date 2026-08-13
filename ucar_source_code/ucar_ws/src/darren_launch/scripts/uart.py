#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import serial
import time
import os

start = 'p'
stop = 's'
ser = serial.Serial('/dev/ttyUSB1',9600,timeout=0.5)
ser.write(start.encode('utf-8')) #penshui
time.sleep(5)
ser.write(stop.encode('utf-8')) #penshui
ser.close()
print(ser.isOpen())
