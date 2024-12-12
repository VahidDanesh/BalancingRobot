#pragma once

// System Configuration
#define SYSTEM_TICK_RATE_HZ 100
#define DEBUG_ENABLED true
#define US_DELAY 3150 // Microseconds delay for 200Hz sampling rate

// Pin Definitions
// Motor 1
#define MOTOR1_STEP_PIN 18
#define MOTOR1_DIR_PIN 19
#define MOTOR1_ENABLE_PIN 27

// Motor 2
#define MOTOR2_STEP_PIN 25
#define MOTOR2_DIR_PIN 26
#define MOTOR2_ENABLE_PIN 14

// MPU6050 Pins
#define MPU_SDA_PIN 21
#define MPU_SCL_PIN 22
#define MPU_INT_PIN 23  // Choose an interrupt-capable pin
#define MPU_I2C_CLOCK 400000UL  
#define MPU_AD0_ADDR 0x69    // AD0 pin is high (0x69) or low (0x68)

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
#define MICROSTEPS 32
#define STEPS_PER_REV 200
#define MAX_SPEED 25000  // steps/second
#define MAX_SPEED_RPM 200 //rpm
#define MAX_ACCELERATION MAX_SPEED*5  // steps/second^2
#define MAX_SPEED_MPS 0.7f



// WiFi Configuration
#define WIFI_SSID "GalaxyS23"
#define WIFI_PASSWORD "123456789S23"

// Web Server Port
#define WEB_SERVER_PORT 80

// Safety limits  
#define MAX_TILT_ANGLE 45.0f  
#define EMERGENCY_STOP_ANGLE 60.0f  

// Control modes  
#define PID_ANGLE 0  
#define PID_POS 1  
#define PID_SPEED 2  


#define WHEEL_RADIUS 0.0325f