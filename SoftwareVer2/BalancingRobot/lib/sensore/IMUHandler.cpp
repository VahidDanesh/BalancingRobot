#include "IMUHandler.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "config.h"


volatile bool IMUHandler::newDataAvailable = false;

// Constructor implementation (if not already in the header)
IMUHandler::IMUHandler() : 
    gyroX(0), gyroY(0), gyroZ(0),
    accelX(0), accelY(0), accelZ(0),
    pitch(0), roll(0),
    previousTime(0),
    gyroXoffset(0), gyroYoffset(0), gyroZoffset(0),
    accelXoffset(0), accelYoffset(0), accelZoffset(0),
    calibrated(false) {
}



void IRAM_ATTR IMUHandler::handleInterrupt() {
    newDataAvailable = true;
}


bool IMUHandler::begin() {
    // Reset I2C bus
    resetI2C();

    // Try to initialize MPU6050 with timeout handling
    if (!initializeMPU()) {
        Serial.println("Failed to initialize MPU6050");
        return false;
    }

    // Configure sensor settings
    if (!configureMPU()) {
        Serial.println("Failed to configure MPU6050");
        return false;
    }

    // Setup interrupt
    setupInterrupt();

    // Initialize timing
    previousTime = micros();

    // Perform initial calibration
    calibrate();

    return true;
}

bool IMUHandler::initializeMPU() {
    for (int retry = 0; retry < I2C_RETRY_COUNT; retry++) {
        if (mpu.begin(MPU_I2C_ADDR, &Wire, 0)) {
            return true;
        }
        delay(50);
        resetI2C();
    }
    return false;
}

bool IMUHandler::configureMPU() {
    bool success = true;

    // Use try-catch block for each configuration attempt
    for (int retry = 0; retry < I2C_RETRY_COUNT && success; retry++) {
        success &= writeRegister(0x6B, 0x00);  // Wake up
        delay(10);
        success &= writeRegister(0x1A, 0x03);  // Configure DLPF
        success &= writeRegister(0x1B, 0x08);  // Gyro Full Scale Range: ±500°/s
        success &= writeRegister(0x1C, 0x10);  // Accel Full Scale Range: ±8g

        if (!success && retry < I2C_RETRY_COUNT - 1) {
            resetI2C();
            delay(50);
        }
    }

    return success;
}

void IMUHandler::resetI2C() {
    Wire.end();
    pinMode(MPU_SDA_PIN, INPUT);
    pinMode(MPU_SCL_PIN, INPUT);
    delay(100);

    // Toggle SCL to release SDA if stuck
    pinMode(MPU_SCL_PIN, OUTPUT);
    for(int i = 0; i < 16; i++) {
        digitalWrite(MPU_SCL_PIN, HIGH);
        delayMicroseconds(3);
        digitalWrite(MPU_SCL_PIN, LOW);
        delayMicroseconds(3);
    }

    // Release pins and restart I2C
    pinMode(MPU_SDA_PIN, INPUT);
    pinMode(MPU_SCL_PIN, INPUT);
    delay(100);

    Wire.begin(MPU_SDA_PIN, MPU_SCL_PIN);
    Wire.setClock(MPU_I2C_CLOCK);
}

bool IMUHandler::writeRegister(uint8_t reg, uint8_t data) {
    Wire.beginTransmission(MPU_I2C_ADDR);
    Wire.write(reg);
    Wire.write(data);
    return (Wire.endTransmission() == 0);
}

bool IMUHandler::readRegisters(uint8_t reg, uint8_t* data, uint8_t length) {
    Wire.beginTransmission(MPU_I2C_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) {
        return false;
    }

    uint8_t bytesReceived = Wire.requestFrom(MPU_I2C_ADDR, length);
    if (bytesReceived != length) {
        return false;
    }

    for (uint8_t i = 0; i < length; i++) {
        data[i] = Wire.read();
    }

    return true;
}

void IMUHandler::update() {
    if (!newDataAvailable) {
        return;
    }

    sensors_event_t a, g, temp;
    bool success = false;

    for (int retry = 0; retry < I2C_RETRY_COUNT && !success; retry++) {
        Wire.beginTransmission(MPU_I2C_ADDR);
        if (Wire.endTransmission(true) == 0) {  // Check if device is responding
            if (mpu.getEvent(&a, &g, &temp)) {
                success = true;
                break;
            }
        }

        if (!success && retry < I2C_RETRY_COUNT - 1) {
            delay(5);
            resetI2C();
        }
    }

    if (!success) {
        Serial.println("Failed to read sensor data after retries");
        return;
    }

    newDataAvailable = false;

    // Update sensor values
    gyroX = g.gyro.x;
    gyroY = g.gyro.y;
    gyroZ = g.gyro.z;
    accelX = a.acceleration.x;
    accelY = a.acceleration.y;
    accelZ = a.acceleration.z;

    applyCalibration();
    calculateAngles();
}



void IMUHandler::setupInterrupt() {
    pinMode(MPU_INT_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(MPU_INT_PIN), handleInterrupt, RISING);
}

void IMUHandler::configureRegisters() {
    // Reset the device first
    Wire.beginTransmission(MPU_I2C_ADDR);
    Wire.write(0x6B);  // PWR_MGMT_1 register
    Wire.write(0x80);  // Reset device
    Wire.endTransmission(true);
    delay(100);  // Wait for reset

    // Wake up the device
    Wire.beginTransmission(MPU_I2C_ADDR);
    Wire.write(0x6B);  // PWR_MGMT_1 register
    Wire.write(0x00);  // Wake up (clear sleep bit)
    Wire.endTransmission(true);
    delay(100);

    // Configure interrupt
    Wire.beginTransmission(MPU_I2C_ADDR);
    Wire.write(0x38);  // INT_ENABLE register
    Wire.write(0x01);  // Enable data ready interrupt
    if (Wire.endTransmission(true) != 0) {
        Serial.println("Failed to configure interrupt");
    }

    // Set sample rate to 1kHz
    Wire.beginTransmission(MPU_I2C_ADDR);
    Wire.write(0x19);  // SMPLRT_DIV register
    Wire.write(0x00);  // 1kHz sample rate
    if (Wire.endTransmission(true) != 0) {
        Serial.println("Failed to set sample rate");
    }

    // Configure digital low pass filter
    Wire.beginTransmission(MPU_I2C_ADDR);
    Wire.write(0x1A);  // CONFIG register
    Wire.write(0x03);  // Set DLPF_CFG to 3 for ~44Hz bandwidth
    if (Wire.endTransmission(true) != 0) {
        Serial.println("Failed to configure DLPF");
    }

    delay(50);  // Wait for all configurations to settle
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

void IMUHandler::calibrate() {
    Serial.println("Calibrating MPU6050... keep the sensor still!");

    const int numSamples = 1000;
    float sumGyroX = 0, sumGyroY = 0, sumGyroZ = 0;
    float sumAccelX = 0, sumAccelY = 0, sumAccelZ = 0;

    // Collect samples
    for (int i = 0; i < numSamples; i++) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        sumGyroX += g.gyro.x;
        sumGyroY += g.gyro.y;
        sumGyroZ += g.gyro.z;
        sumAccelX += a.acceleration.x;
        sumAccelY += a.acceleration.y;
        sumAccelZ += (a.acceleration.z - 9.81); // Remove gravity

        delay(1);
    }

    // Calculate offsets
    gyroXoffset = sumGyroX / numSamples;
    gyroYoffset = sumGyroY / numSamples;
    gyroZoffset = sumGyroZ / numSamples;
    accelXoffset = sumAccelX / numSamples;
    accelYoffset = sumAccelY / numSamples;
    accelZoffset = sumAccelZ / numSamples;

    calibrated = true;
    Serial.println("Calibration complete!");
}

void IMUHandler::applyCalibration() {
    if (calibrated) {
        gyroX -= gyroXoffset;
        gyroY -= gyroYoffset;
        gyroZ -= gyroZoffset;
        accelX -= accelXoffset;
        accelY -= accelYoffset;
        accelZ -= accelZoffset;
    }
}

void IMUHandler::calculateAngles() {
    // Get current time and calculate delta time
    unsigned long currentTime = micros();
    float dt = (currentTime - previousTime) / 1000000.0f; // Convert to seconds
    previousTime = currentTime;

    // Calculate accelerometer angles
    float accelPitch = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * RAD_TO_DEG;
    float accelRoll = atan2(-accelX, accelZ) * RAD_TO_DEG;

    // Integrate gyroscope data
    float gyroPitch = pitch + gyroX * dt;
    float gyroRoll = roll + gyroY * dt;

    // Complementary filter
    pitch = ALPHA * gyroPitch + (1.0f - ALPHA) * accelPitch;
    roll = ALPHA * gyroRoll + (1.0f - ALPHA) * accelRoll;
}

void IMUHandler::printData() {
    Serial.print("Pitch: "); Serial.print(pitch);
    Serial.print(" Roll: "); Serial.print(roll);
    Serial.print(" GyroX: "); Serial.print(gyroX);
    Serial.print(" GyroY: "); Serial.print(gyroY);
    Serial.print(" GyroZ: "); Serial.print(gyroZ);
    Serial.print(" AccelX: "); Serial.print(accelX);
    Serial.print(" AccelY: "); Serial.print(accelY);
    Serial.print(" AccelZ: "); Serial.println(accelZ);
}