# Eco_Bee_IoT

An ESP32-based IoT project designed to automate feeding bees with sugar water while monitoring the surrounding environment.

## Project Overview

This project uses an ESP32 microcontroller connected to a servo motor and multiple sensors to control a valve that dispenses sugar water for bees. Environmental data such as temperature, humidity, light intensity, and distance are collected and sent to a web server for monitoring and control via a mobile app. The system connects to the internet through Wi-Fi.

## Hardware Components

- **ESP32 Dev Board** — main controller with Wi-Fi capabilities
- **Servo Motor** — controls the opening/closing of the feeding valve
- **DHT Sensor** — measures temperature and humidity
- **BH1750** — ambient light sensor
- **HC-SR04** — ultrasonic distance sensor for monitoring liquid level or valve position
- **Other sensors** as included in the project files (check `src/main.cpp`)

## Wiring / Hardware Connections

| Component    | ESP32 Pin      | Notes                          |
|--------------|---------------|--------------------------------|
| Servo Motor  | GPIO (as defined in code) | Controls valve servo          |
| DHT Sensor   | GPIO (as defined in code) | Temperature & humidity sensor |
| BH1750       | I2C (SDA, SCL) | Light intensity sensor          |
| HC-SR04      | GPIO (Trig, Echo) | Distance sensor                 |

*Refer to the `src/main.cpp` for exact GPIO pin assignments.*

## Software Setup

1. Clone or download this repository.

2. Create a `src/secrets.h` file with your Wi-Fi credentials and other private macros:

   ```c++
   #ifndef SECRETS_H
   #define SECRETS_H

   #define WIFI_SSID "your_wifi_ssid"
   #define WIFI_PASSWORD "your_wifi_password"

   // Add other secrets/macros as needed from src/main.cpp

   #endif //SECRETS_H

3. Install PlatformIO and open this project.

4. Build and upload the firmware to your ESP32 board.

## Features and Functionality
- **Automated feeding via servo-controlled valve
- **Real-time monitoring of environmental data (temp, humidity, light, distance)
- **Wi-Fi connectivity to send data to a web server or cloud (Firebase)
- **Remote control via a mobile app (details in app or code)
- **Time synchronization via NTP for scheduled feeding