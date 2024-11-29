#include <Arduino.h>
#include "IMUHandler.h"

IMUHandler imu;

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    Serial.println("MPU6050 Test!");

    if (!imu.begin()) {
        Serial.println("Failed to initialize IMU!");
        while (1) delay(10);
    }

    Serial.println("IMU initialized successfully!");
}

void loop() {
    // Only update when new data is available
    if (imu.dataReady()) {
        imu.update();

        // Print data every 100ms
        static unsigned long lastPrint = 0;
        if (millis() - lastPrint >= 100) {
            imu.printData();
            lastPrint = millis();
        }
    }
}