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

    
    // use same length for all values
    Serial.print("YAW: ");
    Serial.print(yaw, 4);
    Serial.print("  ");
    Serial.print("PITCH: ");
    Serial.print(pitch, 4);
    Serial.print("  ");
    Serial.print("ROLL: ");
    Serial.println(roll, 4);
    


    delay(100); // Update every 100ms
}