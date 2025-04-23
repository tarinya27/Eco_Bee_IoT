#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <ArduinoJson.h>
#include <BH1750.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Firebase.h>
#include <HCSR04.h>
#include <NTPClient.h>
#include <Wire.h>

#include <secrets.h>

#define DHT_PIN 4
#define DHT_TYPE DHT22

#define LIGHT_SDA_PIN 25
#define LIGHT_SCL_PIN 26

#define RAIN_AO_PIN 34

#define DISTANCE_TRIG_PIN 33
#define DISTANCE_ECHO_PIN 35

#define PUMP_SPEED_PIN 14
#define PUMP_EN_1_PIN 13
#define PUMP_EN_2_PIN 12

void log_seperator();

void log_temp_sensor_details();
void log_humidity_sensor_details();
void log_light_sensor_details(BH1750::Mode mode, const char *resolution);

double get_fluid_level(float temperature);

void shutdown();

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "85.254.217.2", GMT_OFFSET, 3600);

Firebase fb(FIREBASE_DATABASE, FIREBASE_AUTH_TOKEN);

DHT_Unified dht(DHT_PIN, DHT_TYPE);

BH1750 lightMeter;

UltraSonicDistanceSensor distanceSensor(DISTANCE_TRIG_PIN, DISTANCE_ECHO_PIN);

void setup() {
    Serial.begin(115200);
    WiFiClass::mode(WIFI_STA);
    WiFi.disconnect();
    delay(1000);

    dht.begin();
    log_temp_sensor_details();
    log_seperator();
    log_humidity_sensor_details();
    log_seperator();

    Wire.begin(LIGHT_SDA_PIN, LIGHT_SCL_PIN);
    lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE);
    log_light_sensor_details(BH1750::CONTINUOUS_HIGH_RES_MODE, "1 lx");
    log_seperator();

    pinMode(PUMP_SPEED_PIN, OUTPUT);
    pinMode(PUMP_EN_1_PIN, OUTPUT);
    pinMode(PUMP_EN_2_PIN, OUTPUT);
    analogWrite(PUMP_SPEED_PIN, 0);
    digitalWrite(PUMP_EN_1_PIN, LOW);
    digitalWrite(PUMP_EN_2_PIN, LOW);

    Serial.printf("Connecting to: %s", WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFiClass::status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nConnected to WiFi");
    Serial.printf("IP address: %s\n", WiFi.localIP().toString().c_str());
    log_seperator();

    timeClient.begin();
    Serial.println("Syncing time...");
    if (timeClient.forceUpdate()) {
        Serial.println("Time synced");
        Serial.printf("Datetime: %s\n", timeClient.getFormattedTime().c_str());
    } else {
        Serial.println("Failed to sync time");
        shutdown();
    }
    log_seperator();
}

void loop() {
    delay(2000);

    JsonDocument update;

    timeClient.update();
    Serial.printf("Timestamp: %ld\n", timeClient.getEpochTime());
    update["timestamp"] = timeClient.getEpochTime();

    sensors_event_t event;
    dht.temperature().getEvent(&event);
    if (isnan(event.temperature)) {
        Serial.println("Error reading temperature!");
        update["temperature"] = nullptr;
    } else {
        Serial.printf("Temperature: %.2f °C\n", event.temperature);
        update["temperature"] = event.temperature;
    }
    dht.humidity().getEvent(&event);
    if (isnan(event.relative_humidity)) {
        Serial.println("Error reading humidity!");
        update["humidity"] = nullptr;
    } else {
        Serial.printf("Humidity: %.2f %%\n", event.relative_humidity);
        update["humidity"] = event.relative_humidity;
    }

    double rainIntensity = 100 - analogRead(RAIN_AO_PIN) / 4095.0 * 100;
    Serial.printf("Rain intensity: %.2f\n", rainIntensity);
    update["rainIntensity"] = rainIntensity;

    float lux = lightMeter.readLightLevel();
    Serial.printf("Light intensity: %.2f lx\n", lux);
    update["lightIntensity"] = lux;

    double fluidLevel = get_fluid_level(event.temperature);
    if (fluidLevel == -1) {
        update["fluidLevel"] = nullptr;
    } else {
        update["fluidLevel"] = fluidLevel;
    }

    String updateJson;
    serializeJson(update, updateJson);
    fb.pushJson(FIREBASE_DATA_PATH, updateJson);

    String pumpData = fb.getJson(FIREBASE_PUMP_PATH);
    if (pumpData != "null") {
        fb.remove(FIREBASE_PUMP_PATH);
        Serial.printf("Pump data: %s\n", pumpData.c_str());
        JsonDocument pump;
        DeserializationError error = deserializeJson(pump, pumpData);
        if (error) {
            Serial.println("Failed to parse pump data");
            log_seperator();
            return;
        }
        double quantity = pump["quantity"];
        if (quantity > FLUID_MAX_LEVEL * FLUID_CROSS_SECTION) {
            Serial.println("Quantity exceeds maximum level");
            log_seperator();
            return;
        }
        double current_quantity = get_fluid_level(event.temperature) * FLUID_CROSS_SECTION;
        while (current_quantity < quantity) {
            Serial.printf("Current quantity: %.2f\n", current_quantity);
            analogWrite(PUMP_SPEED_PIN, 255);
            digitalWrite(PUMP_EN_1_PIN, HIGH);
            digitalWrite(PUMP_EN_2_PIN, LOW);
            delay(500);
            current_quantity = get_fluid_level(event.temperature) * FLUID_CROSS_SECTION;
        }
        digitalWrite(PUMP_EN_1_PIN, LOW);
        digitalWrite(PUMP_EN_2_PIN, LOW);
        analogWrite(PUMP_SPEED_PIN, 0);
    } else {
        Serial.println("No pump data");
    }

    log_seperator();
}

double get_fluid_level(const float temperature) {
    double distance;
    if (isnan(temperature)) {
        distance = distanceSensor.measureDistanceCm();
    } else {
        distance = distanceSensor.measureDistanceCm(temperature);
    }
    if (distance == -1) {
        Serial.println("Error reading distance sensor!");
        return -1;
    }
    Serial.printf("Fluid level: %.2f cm\n", FLUID_MAX_LEVEL - distance);
    return FLUID_MAX_LEVEL - distance;
}

void log_temp_sensor_details() {
    sensor_t sensor;
    dht.temperature().getSensor(&sensor);
    Serial.printf("Temperature Sensor\n");
    Serial.printf("Sensor:       %s\n", sensor.name);
    Serial.printf("Driver Ver:   %d\n", sensor.version);
    Serial.printf("Unique ID:    %d\n", sensor.sensor_id);
    Serial.printf("Max Value:    %.2f °C\n", sensor.max_value);
    Serial.printf("Min Value:    %.2f °C\n", sensor.min_value);
    Serial.printf("Resolution:   %.2f °C\n", sensor.resolution);
}

void log_humidity_sensor_details() {
    sensor_t sensor;
    dht.humidity().getSensor(&sensor);
    Serial.printf("Humidity Sensor\n");
    Serial.printf("Sensor:       %s\n", sensor.name);
    Serial.printf("Driver Ver:   %d\n", sensor.version);
    Serial.printf("Unique ID:    %d\n", sensor.sensor_id);
    Serial.printf("Max Value:    %.2f %%\n", sensor.max_value);
    Serial.printf("Min Value:    %.2f %%\n", sensor.min_value);
    Serial.printf("Resolution:   %.2f %%\n", sensor.resolution);
}

void log_light_sensor_details(const BH1750::Mode mode, const char *resolution) {
    Serial.printf("Light Sensor\n");
    Serial.printf("Sensor:       BH1750\n");
    Serial.printf("Mode:         %d\n", mode);
    Serial.printf("Resolution:   %s\n", resolution);
}

void log_seperator() { Serial.printf("----------------------------------------------------\n"); }

void shutdown() {
    Serial.println("Shutting down...");
    Serial.flush();
    Serial.end();
    WiFi.disconnect();
    esp_deep_sleep_start();
}
