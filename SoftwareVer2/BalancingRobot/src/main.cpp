// main.cpp
#include <Arduino.h>
#include "IMUHandler.h"

IMUHandler imu;

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("MPU6050 Test!");

    if (!imu.begin()) {
        Serial.println("Failed to initialize IMU!");
        while (1) {
            delay(10);
        }
    }

    // Calibrate the sensor
    imu.calibrate();
}

void loop() {
    imu.update();

    static unsigned long lastPrint = 0;
    if (millis() - lastPrint >= 100) {  // Print every 100ms
        imu.printData();
        lastPrint = millis();
    }
}