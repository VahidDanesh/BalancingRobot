#pragma once

// System Configuration
#define SYSTEM_TICK_RATE_HZ 100
#define DEBUG_ENABLED true

// Pin Definitions
// Motor 1
#define MOTOR1_STEP_PIN 18
#define MOTOR1_DIR_PIN 19
#define MOTOR1_ENABLE_PIN 27

// Motor 2
#define MOTOR2_STEP_PIN 26
#define MOTOR2_DIR_PIN 27
#define MOTOR2_ENABLE_PIN 14

// MPU6050 Pins
#define MPU_SDA_PIN 21
#define MPU_SCL_PIN 22
#define MPU_INT_PIN 23  // Choose an interrupt-capable pin
#define MPU_AD0_PIN -1  // AD0 pin is high (0x69) or low (0x68)
#define MPU_I2C_CLOCK 100000  // Reduced to 100kHz for stability
#define MPU_I2C_ADDR 0x68     // MPU6050 I2C address

// I2C Communication settings
#define I2C_TIMEOUT_MS 50     // I2C timeout in milliseconds
#define I2C_RETRY_COUNT 3     // Number of retries for I2C operations

// Debug options
#define SERIAL_DEBUG true
#define SERIAL_BAUD 115200

// Built-in LED
#define LED_PIN 2

// Neopixel LED
#define NEOPIXEL_PIN 16

// Motor Configuration
#define MICROSTEPS 16
#define STEPS_PER_REV 200
#define MAX_SPEED 20000  // steps/second
#define MAX_ACCELERATION 200000  // steps/second^2

// PID Constants
#define KP_DEFAULT 1.0
#define KI_DEFAULT 0.0
#define KD_DEFAULT 0.0

// WiFi Configuration
#define WIFI_SSID "GalaxyS23"
#define WIFI_PASSWORD "123456789S23"

// Web Server Port
#define WEB_SERVER_PORT 80

// Safety Limits
#define MAX_TILT_ANGLE 30.0f  // degrees
#define EMERGENCY_STOP_ANGLE 60 .0f  // degrees