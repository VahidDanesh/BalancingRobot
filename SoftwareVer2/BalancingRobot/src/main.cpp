#include "IMUHandler.h"
#include <config.h>

IMUHandler& imu = IMUHandler::getInstance();

void setup() {
    Serial.begin(SERIAL_BAUD);
    
    imu.initialize();
    imu.calibrate();
}

void loop() {
    imu.update();

    float yaw = imu.getYaw();
    float pitch = imu.getPitch();
    float roll = imu.getRoll();

    // print the angles with same char length for better readability
    Serial.print("Yaw:   ");
    Serial.print(yaw, 2);
    Serial.print("   ");
    Serial.print("Pitch: ");
    Serial.print(pitch, 2);
    Serial.print("   ");
    Serial.print("Roll:  ");
    Serial.println(roll, 2);


    delay(100); // Update every 100ms
}