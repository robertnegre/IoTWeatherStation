#!/usr/bin/python
 
import time
import board
import adafruit_dht
from gpiozero import DigitalInputDevice
import Adafruit_BMP.BMP085 as BMP085
import logging
import RPi.GPIO as GPIO
 
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
 
# Initialize GPIO for LEDs
#GPIO.setmode(GPIO.BOARD)
GPIO.setup(13, GPIO.OUT)  # Red LED
GPIO.setup(5, GPIO.OUT) # Blue LED
GPIO.setup(6, GPIO.OUT) # Green LED
GPIO.setup(19, GPIO.OUT) # Yellow LED
 
# Create PWM instances for each LED
red_led = GPIO.PWM(13, 1000)  # 1000 Hz frequency
blue_led = GPIO.PWM(5, 1000)  # 1000 Hz frequency
green_led = GPIO.PWM(6, 1000)  # 1000 Hz frequency
yellow_led = GPIO.PWM(19, 1000)  # 1000 Hz frequency
 
# Start with 0% duty cycle
red_led.start(0)
blue_led.start(0)
green_led.start(0)
yellow_led.start(0)
 
def red_led_on():
    red_led.ChangeDutyCycle(100)
    blue_led.ChangeDutyCycle(0)
    green_led.ChangeDutyCycle(0)
    yellow_led.ChangeDutyCycle(0)
 
def blue_led_on():
    red_led.ChangeDutyCycle(0)
    blue_led.ChangeDutyCycle(100)
    green_led.ChangeDutyCycle(0)
    yellow_led.ChangeDutyCycle(0)
 
def green_led_on():
    red_led.ChangeDutyCycle(0)
    blue_led.ChangeDutyCycle(0)
    green_led.ChangeDutyCycle(100)
    yellow_led.ChangeDutyCycle(0)
 
def yellow_led_on():
    red_led.ChangeDutyCycle(0)
    blue_led.ChangeDutyCycle(0)
    green_led.ChangeDutyCycle(0)
    yellow_led.ChangeDutyCycle(100)
 
def all_leds_off():
    red_led.ChangeDutyCycle(0)
    blue_led.ChangeDutyCycle(0)
    green_led.ChangeDutyCycle(0)
    yellow_led.ChangeDutyCycle(0)
 
def determine_forecast(humidity, temperature, pressure, altitude, rain):
    #if temperature > 30 and humidity > 70:
    if humidity > 70:
        # Forecast 1: Hot & Humid Weather
        red_led_on()
        print("RED LED ON   Hot & Humid Weather")
        red_led_status = True
    #elif temperature < 15 and humidity < 30:
    elif 65 <= humidity < 70:
        # Forecast 2: Cool & Dry Weather
        blue_led_on()
        print("BLUE LED ON    Cool & Dry Weather")
        blue_led_status = True
    #elif pressure < 100000 and rain and humidity > 80:
    elif pressure > 100000 and rain and humidity > 60:
        # Forecast 3: Stormy Weather
        green_led_on()
        print("GREEN LED ON    Stormy Weather")
        green_led_status = True
    elif 20 <= temperature <= 29 and 30 <= humidity <= 70 and not rain:
        # Forecast 4: Mild & Pleasant Weather
        yellow_led_on()
        print("YELLOW LED ON   Mild & Pleasant Weather")
        yellow_led_status = True
 
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
 
    if humidity is not None and temperature is not None and pressure is not None and altitude is not None and sealevel_pressure is not None and rain is not None:
        print("Humidity: {}%".format(humidity))
        print('Temperature: {0:0.2f} *C'.format(temperature))
        print('Pressure: {0:0.2f} Pa'.format(pressure))
        print('Altitude: {0:0.2f} m'.format(altitude))
        print('Sealevel Pressure: {0:0.2f} Pa'.format(sealevel_pressure))
        if rain:
            print("It's raining!")
        else:
            print("It's NOT raining!")
 
        determine_forecast(humidity, temperature, pressure, altitude, rain)
 
    print()
    print()
    time.sleep(5)  # Delay for 2 seconds between sensor readings
