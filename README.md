# IoTWeatherStation

Repo for Introduction to IoT and Cloud Architectures Project.
Team Members: Robert Negre, Vlad Mocan. 


Our IoT weather monitoring system is based on a Raspberry Pi 4 with 4GB RAM and includes various sensors: the DHT11 for temperature and humidity, the BMP180 for temperature, pressure, and altitude, and the YL-83 for precipitation. The system features four LEDs (yellow, blue, red, green) and a buzzer for user feedback.

The system monitors temperature, humidity, atmospheric pressure, altitude, and precipitation, using this data to generate four weather forecasts indicated by the LEDs. Yellow represents sunny and mild weather, blue indicates cool and dry, red denotes hot and warm, and green warns of stormy weather, accompanied by a buzzer alarm.

Data is transmitted to Google Firebase's Realtime Database for real-time updates and historical storage. The database records sensor data, LED states, and buzzer status with timestamps.

A complementary website provides real-time and historical weather data, with graphical trends for temperature and humidity. A mobile app offers similar functionalities, including real-time updates and alerts for stormy weather, and allows users to silence the buzzer.

Website Repo: https://github.com/pseudocod/IIOTCA-Web-Project
Mobile App Repo: https://github.com/pseudocod/IIOTCA-Mobile-Project
