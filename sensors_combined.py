#!/usr/bin/python

import time
import board
import adafruit_dht
from gpiozero import DigitalInputDevice
import Adafruit_BMP.BMP085 as BMP085
import logging

# Setup logging
logging.basicConfig(level=logging.INFO)

# Initialize the DHT11 sensor on GPIO pin 27
dhtDevice = adafruit_dht.DHT11(board.D27)

# Initialize the rain sensor as a digital input device on GPIO pin 17
rain_sensor = DigitalInputDevice(17)

try:
    # Initialize the BMP180 sensor on I2C bus 1
    bmpSensor = BMP085.BMP085(busnum=1)
    time.sleep(1)  # Delay to ensure sensor is ready
except OSError as e:
    logging.error("Error initializing BMP180 sensor: %s", e)
    bmpSensor = None

def read_dht_sensor():
    try:
        humidity = dhtDevice.humidity
        return humidity
    except RuntimeError as error:
        logging.error("Error reading DHT11 sensor: %s", error.args[0])
        return None

def read_rain_sensor():
    try:
        return not rain_sensor.is_active
    except Exception as e:
        logging.error("Error reading rain sensor: %s", e)
        return None

def read_bmp_sensor():
    if bmpSensor is None:
        return None, None, None, None
    try:
        temperature = bmpSensor.read_temperature()
        pressure = bmpSensor.read_pressure()
        altitude = bmpSensor.read_altitude()
        sealevel_pressure = bmpSensor.read_sealevel_pressure()
        return temperature, pressure, altitude, sealevel_pressure
    except Exception as e:
        logging.error("Error reading BMP180 sensor: %s", e)
        return None, None, None, None

while True:
    humidity = read_dht_sensor()
    rain = read_rain_sensor()
    temperature, pressure, altitude, sealevel_pressure = read_bmp_sensor()

    if humidity is not None:
        print("Humidity: {}%".format(humidity))
            
    if temperature is not None and pressure is not None and altitude is not None and sealevel_pressure is not None:
        print('Temperature: {0:0.2f} *C'.format(temperature))
        print('Pressure: {0:0.2f} Pa'.format(pressure))
        print('Altitude: {0:0.2f} m'.format(altitude))
        print('Sealevel Pressure: {0:0.2f} Pa'.format(sealevel_pressure))
            
    if rain is not None:
        if rain:
            print("It's raining!")
        else:
            print("It's NOT raining!")
    
    print()
    time.sleep(2)  # Delay for 2 seconds between sensor readings
