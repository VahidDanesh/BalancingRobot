// config.h
#ifndef CONFIG_H
#define CONFIG_H

// WiFi Credentials
const char* WIFI_SSID = "GalaxyS23";       // Replace with your WiFi SSID
const char* WIFI_PASSWORD = "123456789S23"; // Replace with your WiFi password

// HTTP Server Credentials
const char* HTTP_USERNAME = "admin";
const char* HTTP_PASSWORD = "admin";

// Pin Definitions
const int MOT_ENABLE_PIN = 27;
const int LED_PIN = 2;
const int MOTOR_CURRENT_PIN = 25;
const int BATT_VOLTAGE_PIN = 34;

// Other Configurations
const char* HOSTNAME = "balancingrobot";
const float SPEED_FACTOR = 0.5;
const float STEER_FACTOR = 1.0;

#endif