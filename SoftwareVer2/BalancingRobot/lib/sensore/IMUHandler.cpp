// IMUHandler.cpp
#include "IMUHandler.h"

IMUHandler::IMUHandler() : 
    pitch(0), roll(0),
    pitchOffset(0), rollOffset(0),
    gyroXoffset(0), gyroYoffset(0),
    previousTime(0) {
}

bool IMUHandler::begin() {
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        return false;
    }

    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);  // Changed to 2G for better resolution
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);       // Changed to 250 deg/s
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    previousTime = micros();
    return true;
}

void IMUHandler::update() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    float dt = (micros() - previousTime) / 1000000.0f;
    previousTime = micros();

    // Calculate accelerometer angles
    float accelPitch = atan2(a.acceleration.y, sqrt(a.acceleration.x * a.acceleration.x + 
                            a.acceleration.z * a.acceleration.z)) * RAD_TO_DEG;
    float accelRoll = atan2(-a.acceleration.x, a.acceleration.z) * RAD_TO_DEG;

    // Apply gyro offsets
    float gyroX = g.gyro.x - gyroXoffset;
    float gyroY = g.gyro.y - gyroYoffset;

    // Integrate gyroscope data
    float gyroPitch = pitch + gyroX * dt;
    float gyroRoll = roll + gyroY * dt;

    // Complementary filter
    pitch = (ALPHA * gyroPitch + (1.0f - ALPHA) * accelPitch) - pitchOffset;
    roll = (ALPHA * gyroRoll + (1.0f - ALPHA) * accelRoll) - rollOffset;

    // Apply zero threshold
    if (abs(pitch) < ZERO_THRESHOLD) pitch = 0;
    if (abs(roll) < ZERO_THRESHOLD) roll = 0;
}

void IMUHandler::calibrate() {
    Serial.println("Calibrating MPU6050...");
    Serial.println("Keep the sensor still and level!");
    delay(2000); // Give time to place the sensor

    float sumPitch = 0;
    float sumRoll = 0;
    float sumGyroX = 0;
    float sumGyroY = 0;

    // Collect samples
    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        // Calculate accelerometer angles
        float accelPitch = atan2(a.acceleration.y, sqrt(a.acceleration.x * a.acceleration.x + 
                                a.acceleration.z * a.acceleration.z)) * RAD_TO_DEG;
        float accelRoll = atan2(-a.acceleration.x, a.acceleration.z) * RAD_TO_DEG;

        sumPitch += accelPitch;
        sumRoll += accelRoll;
        sumGyroX += g.gyro.x;
        sumGyroY += g.gyro.y;

        if (i % 100 == 0) {
            Serial.print("Calibrating... ");
            Serial.print(i * 100 / CALIBRATION_SAMPLES);
            Serial.println("%");
        }
        delay(5);
    }

    // Calculate offsets
    pitchOffset = sumPitch / CALIBRATION_SAMPLES;
    rollOffset = sumRoll / CALIBRATION_SAMPLES;
    gyroXoffset = sumGyroX / CALIBRATION_SAMPLES;
    gyroYoffset = sumGyroY / CALIBRATION_SAMPLES;

    // Reset angles
    pitch = 0;
    roll = 0;

    Serial.println("Calibration complete!");
    Serial.print("Pitch offset: "); Serial.println(pitchOffset);
    Serial.print("Roll offset: "); Serial.println(rollOffset);
    Serial.print("Gyro X offset: "); Serial.println(gyroXoffset);
    Serial.print("Gyro Y offset: "); Serial.println(gyroYoffset);
}

void IMUHandler::printData() {
    Serial.print("Pitch: "); Serial.print(pitch, 3); // Increased decimal precision
    Serial.print(" Roll: "); Serial.println(roll, 3);
}