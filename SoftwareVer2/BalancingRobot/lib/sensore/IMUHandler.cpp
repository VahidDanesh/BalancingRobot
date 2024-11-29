#include "IMUHandler.h"

volatile bool IMUHandler::newDataAvailable = false;

void IRAM_ATTR IMUHandler::handleInterrupt() {
    newDataAvailable = true;
}

bool IMUHandler::begin() {
    // Initialize I2C with higher clock speed
    Wire.begin(MPU_SDA_PIN, MPU_SCL_PIN);
    Wire.setClock(MPU_I2C_CLOCK);

    // Configure AD0 pin if specified
    #if MPU_AD0_PIN != -1
        pinMode(MPU_AD0_PIN, OUTPUT);
        digitalWrite(MPU_AD0_PIN, LOW);  // Set to GND for address 0x68
    #endif

    // Try to initialize MPU6050
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        return false;
    }

    // Setup sensor ranges and filters
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    // Configure interrupt
    setupInterrupt();

    // Additional register configuration
    configureRegisters();

    // Small delay for sensor to settle
    delay(100);

    // Initialize timing
    previousTime = micros();

    // Perform initial calibration
    calibrate();

    return true;
}

void IMUHandler::setupInterrupt() {
    pinMode(MPU_INT_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(MPU_INT_PIN), handleInterrupt, RISING);
}

void IMUHandler::configureRegisters() {
    // Access registers directly for advanced configuration
    Wire.beginTransmission(0x68); // MPU6050 address
    Wire.write(0x6B);  // PWR_MGMT_1 register
    Wire.write(0x00);  // Wake up the MPU-6050
    Wire.endTransmission(true);

    // Configure interrupt
    Wire.beginTransmission(0x68);
    Wire.write(0x38);  // INT_ENABLE register
    Wire.write(0x01);  // Enable data ready interrupt
    Wire.endTransmission(true);

    // Set sample rate to 1kHz
    Wire.beginTransmission(0x68);
    Wire.write(0x19);  // SMPLRT_DIV register
    Wire.write(0x00);  // 1kHz sample rate
    Wire.endTransmission(true);
}

void IMUHandler::update() {
    if (!newDataAvailable) {
        return;  // No new data available
    }

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Reset the flag
    newDataAvailable = false;

    // Store raw values
    gyroX = g.gyro.x;
    gyroY = g.gyro.y;
    gyroZ = g.gyro.z;
    accelX = a.acceleration.x;
    accelY = a.acceleration.y;
    accelZ = a.acceleration.z;

    // Apply calibration offsets
    applyCalibration();

    // Calculate filtered angles
    calculateAngles();
}