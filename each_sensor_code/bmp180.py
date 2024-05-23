#!/usr/bin/python

import time
import Adafruit_BMP.BMP085 as BMP085

sensor = BMP085.BMP085(busnum=1)

while True:
    print('Temp = {0:0.2f} *C'.format(sensor.read_temperature()))
    print('Pressure = {0:0.2f} Pa'.format(sensor.read_pressure()))
    print('Altitude = {0:0.2f} m'.format(sensor.read_altitude()))
    print('Sealevel Pressure = {0:0.2f} Pa'.format(sensor.read_sealevel_pressure()))
    print("\n\n")  # Adding two empty lines for separation
    time.sleep(2)  # Delay for 5 seconds
