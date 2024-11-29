// IMUHandler.h
#pragma once

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

class IMUHandler {
public:
    IMUHandler();
    bool begin();
    void update();
    void calibrate();

    // Getters
    float getPitch() const { return pitch; }
    float getRoll() const { return roll; }

    // Debug
    void printData();

private:
    Adafruit_MPU6050 mpu;

    // Filtered angles
    float pitch;
    float roll;

    // Offsets
    float pitchOffset;
    float rollOffset;
    float gyroXoffset;
    float gyroYoffset;

    // Previous time for complementary filter
    unsigned long previousTime;

    // Filter constants
    static constexpr float ALPHA = 0.96f;
    static constexpr float ZERO_THRESHOLD = 0.05f; // Threshold for considering angle as zero
    static constexpr int CALIBRATION_SAMPLES = 500; // Increased number of samples
};