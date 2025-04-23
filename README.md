# Bee Feeder IoT

- An ESP32-based IoT project to feed bees with sugar water.
- The project uses the ESP32 microcontroller and the Arduino framework.
- The ESP32 is connected to a servo motor to open and close a valve for feeding the bees.
- The ESP32 is also connected to multiple sensors to monitor the environment and send data to a web server.
- The ESP32 connects to the internet via Wi-Fi and can be controlled through a mobile app.

## Setup

- Create the src/secrets.h file with the following content:

```c++
#ifndef SECRETS_H
#define SECRETS_H

#define WIFI_SSID "your_wifi_ssid"
#define WIFI_PASSWORD "your_wifi_password"
// Define other Macros from src/main.cpp

#endif //SECRETS_H
```
