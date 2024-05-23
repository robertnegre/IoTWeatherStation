#!/usr/bin/python

import time
import board
import adafruit_dht
from gpiozero import DigitalInputDevice
import Adafruit_BMP.BMP085 as BMP085

# Initialize the DHT11 sensor on GPIO pin 27
dhtDevice = adafruit_dht.DHT11(board.D27)

# Initialize the rain sensor as a digital input device on GPIO pin 17
rain_sensor = DigitalInputDevice(17)

# Initialize the BMP180 sensor on I2C bus 1
bmpSensor = BMP085.BMP085(busnum=1)

while True:
    try:
        # Read data from DHT11 sensor
        temperature_c = dhtDevice.temperature
        humidity = dhtDevice.humidity
        print("DHT11 Sensor:")
        print("Temperature: {:.1f} C    Humidity: {}% ".format(temperature_c, humidity))
        print("\n") 

    except RuntimeError as error:
        print("Error reading DHT11 sensor:", error.args[0])

    try:
        # Read data from rain sensor
        if rain_sensor.is_active:
            print("Rain Sensor: No rain detected.")
            print("\n") 
        else:
            print("Rain Sensor: Rain detected!")
            print("\n") 

    except Exception as e:
        print("Error reading rain sensor:", e)

    try:
        # Read data from BMP180 sensor
        print("BMP180 Sensor:")
        print('Temperature: {0:0.2f} *C'.format(bmpSensor.read_temperature()))
        print('Pressure: {0:0.2f} Pa'.format(bmpSensor.read_pressure()))
        print('Altitude: {0:0.2f} m'.format(bmpSensor.read_altitude()))
        print('Sealevel Pressure: {0:0.2f} Pa'.format(bmpSensor.read_sealevel_pressure()))
        print("\n") 

    except Exception as e:
        print("Error reading BMP180 sensor:", e)

    print("\n")  # Adding a new line between data from each sensor
    time.sleep(2)  # Delay for 2 seconds between sensor readings
