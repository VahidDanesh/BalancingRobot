#include "IMUHandler.h"
#include "config.h"
#include <Wire.h>
#include <Arduino.h>

// Singleton instance
IMUHandler& IMUHandler::getInstance() {
    static IMUHandler instance;
    return instance;
}

// Free function for interrupt handling
void IMUHandler::dmpDataReady() {
    IMUHandler::getInstance().mpuInterrupt = true;
}

IMUHandler::IMUHandler() 
    : mpu(MPU_AD0_ADDR),
      dmpReady(false), 
      calibrated(false), 
      yawOffset(0), 
      pitchOffset(0), 
      rollOffset(0),
      mpuIntStatus(false) {}

void IMUHandler::initialize() {
    Wire.begin(MPU_SDA_PIN, MPU_SCL_PIN, MPU_I2C_CLOCK);
    Wire.setClock(MPU_I2C_CLOCK); // Set I2C clock to 400kHz for faster communication

    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(MPU_INT_PIN, INPUT);

    Serial.println(F("Testing device connections..."));

    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed!");
        while (1); // Halt execution if the sensor is not connected
    }

    Serial.println("MPU6050 connection successful.");

    // Initialize DMP
    devStatus = mpu.dmpInitialize();

    if (devStatus == 0) {
        // Enable DMP
        Serial.println("Enabling DMP...");
        mpu.setDMPEnabled(true);

        // Enable interrupt detection
        Serial.print("Enabling interrupt detection on pin ");
        Serial.println(MPU_INT_PIN);
        attachInterrupt(digitalPinToInterrupt(MPU_INT_PIN), IMUHandler::dmpDataReady, RISING);

        // Get expected DMP packet size
        packetSize = mpu.dmpGetFIFOPacketSize();
        dmpReady = true;
        Serial.println("DMP ready! Waiting for first interrupt...");
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print("DMP Initialization failed (code ");
        Serial.print(devStatus);
        Serial.println(")");
    }
}

void IMUHandler::calibrate(bool force) {
    if (!dmpReady) {
        Serial.println("Error: DMP not initialized. Call initialize() first.");
        return;
    }

    Serial.println("Starting calibration...");
    Serial.println("Please hold the sensor still and level.");

    delay(3150); // Give the user time to stabilize the sensor

    if (force) {
    // Calibrate accelerometer and gyroscope
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();

    delay(3150);


    // Save offsets

    mpu.setXAccelOffset(mpu.getXAccelOffset());
    mpu.setYAccelOffset(mpu.getYAccelOffset());
    mpu.setZAccelOffset(mpu.getZAccelOffset());
    mpu.setXGyroOffset(mpu.getXGyroOffset());
    mpu.setYGyroOffset(mpu.getYGyroOffset());
    mpu.setZGyroOffset(mpu.getZGyroOffset());
    } else {
        // use the calibration offset from PID calibration
        // see https://github.com/jrowberg/i2cdevlib/blob/master/Arduino/MPU6050/examples/IMU_Zero/IMU_Zero.ino
        
        mpu.setXAccelOffset(-2321);
        mpu.setYAccelOffset(309);
        mpu.setZAccelOffset(1047);
        mpu.setXGyroOffset(169);
        mpu.setYGyroOffset(-3);
        mpu.setZGyroOffset(-1);
    }

    // print prompt to serial monitor
    Serial.println(force ? "Offsets are set." : "Offsets are set. Using PID calibration offsets.");

    
    // Read initial yaw, pitch, and roll to calculate offsets
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        yawOffset = ypr[0];
        pitchOffset = ypr[1];
        rollOffset = ypr[2];
    }

    calibrated = true;
    Serial.println("Calibration complete.");
}

void IMUHandler::update() {
    if (!dmpReady) {
        Serial.println("Error: DMP not initialized. Call initialize() first.");
        return;
    }

    if (!calibrated) {
        Serial.println("Error: Sensor not calibrated. Call calibrate() first.");
        return;
    }

    // Check if interrupt flag is set
    if (mpuInterrupt) {
        mpuInterrupt = false; // Clear the interrupt flag

        // Read a packet from the FIFO
        if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

            // Subtract offsets to get calibrated angles
            ypr[0] -= yawOffset;  // Yaw
            ypr[1] -= pitchOffset; // Pitch
            ypr[2] -= rollOffset;  // Roll
            // assume zero if less than 1 degree
            if (abs(ypr[0]) < 1 * DEG_TO_RAD) ypr[0] = 0;
            if (abs(ypr[1]) < 1 * DEG_TO_RAD) ypr[1] = 0;
            if (abs(ypr[2]) < 1 * DEG_TO_RAD) ypr[2] = 0;

            // Calculate angular rates (yawRate, rollRate)  
            uint32_t currentTime = millis();  
            float deltaTime = (currentTime - lastTime) / 1000.0; // Convert to seconds  

            if (deltaTime > 0) {  
                yprRate[0] = (ypr[0] - lastYPR[0]) / deltaTime; // Yaw rate  
                yprRate[1] = (ypr[1] - lastYPR[1]) / deltaTime; // Pitch rate  
                yprRate[2] = (ypr[2] - lastYPR[2]) / deltaTime; // Roll rate  
            }  

            // Update last values  
            lastYPR[0] = ypr[0];  
            lastYPR[1] = ypr[1];  
            lastYPR[2] = ypr[2];  
            lastTime = currentTime; 
        }
    }
}

float IMUHandler::getYaw() const {
    return ypr[0] ;
}

float IMUHandler::getPitch() const {
    return ypr[1] ; 
}

float IMUHandler::getRoll() const {
    return ypr[2] ;
}

float IMUHandler::getYawRate() const {  
    return yprRate[0] ;  
}

float IMUHandler::getPitchRate() const {  
    return yprRate[1] ;
}

float IMUHandler::getRollRate() const {  
    return yprRate[2] ;  
}  