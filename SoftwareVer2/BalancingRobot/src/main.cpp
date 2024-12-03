#include "IMUHandler.h"
#include <config.h>

IMUHandler& imu = IMUHandler::getInstance();


void printAlignedValue(const char* label, float value, int width);


void setup() {
    Serial.begin(SERIAL_BAUD);
    
    imu.initialize();
    imu.calibrate(true);
}

void loop() {
    imu.update();

    float yaw = imu.getYaw();
    float pitch = imu.getPitch();
    float roll = imu.getRoll();

    
    // Print formatted values
    printAlignedValue("YAW: ", yaw, 8);    // 8 characters wide
    printAlignedValue("PITCH: ", pitch, 8); // 8 characters wide
    printAlignedValue("ROLL: ", roll, 8);   // 8 characters wide
    Serial.println();



    delay(100); // Update every 100ms
}





void printAlignedValue(const char* label, float value, int width) {
    Serial.print(label);
    if (value >= 0) {
        Serial.print(" "); // Add a space for positive values to align with negative ones
    }
    // Print the value with fixed width
    Serial.print(value, 2); 
    int valueLength = String(value, 2).length();
    // Add extra spaces to pad the output if it's shorter than the width
    for (int i = valueLength; i < width; i++) {
        Serial.print(" ");
    }
}