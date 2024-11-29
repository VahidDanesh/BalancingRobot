#pragma once

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "config.h"

class IMUHandler {
public:
    IMUHandler();
    bool begin();
    void update();

    // Getters for filtered angles and raw data
    float getPitch() const { return pitch; }
    float getRoll() const { return roll; }
    float getGyroX() const { return gyroX; }
    float getGyroY() const { return gyroY; }
    float getGyroZ() const { return gyroZ; }
    float getAccelX() const { return accelX; }
    float getAccelY() const { return accelY; }
    float getAccelZ() const { return accelZ; }

    // Calibration
    void calibrate();
    bool isCalibrated() const { return calibrated; }

    // Debug
    void printData();
    
    // Interrupt handling
    static void IRAM_ATTR handleInterrupt();
    bool dataReady() const { return newDataAvailable; }

private:
    Adafruit_MPU6050 mpu;
    static volatile bool newDataAvailable;

    // Raw sensor data
    float gyroX, gyroY, gyroZ;
    float accelX, accelY, accelZ;

    // Filtered angles
    float pitch;
    float roll;

    // Previous time for integration
    unsigned long previousTime;

    // Calibration offsets
    float gyroXoffset, gyroYoffset, gyroZoffset;
    float accelXoffset, accelYoffset, accelZoffset;
    bool calibrated;

    // Filter coefficient (0 to 1)
    // Higher value gives more weight to accelerometer
    static constexpr float ALPHA = 0.96;

    // Helper functions
    void calculateAngles();
    void applyCalibration();
    void setupInterrupt();
    void configureRegisters();
    void resetI2C();
    bool configureMPU();
    bool initializeMPU();
    bool writeRegister(uint8_t reg, uint8_t value);
    bool readRegisters(uint8_t reg, uint8_t* data, uint8_t length);
};