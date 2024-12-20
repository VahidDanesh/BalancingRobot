#ifndef IMUHANDLER_H
#define IMUHANDLER_H

#include "MPU6050_6Axis_MotionApps20.h"
#include <config.h>

class IMUHandler {
public:
    static IMUHandler& getInstance(); // Singleton instance
    void initialize();
    void calibrate(bool force=false);
    void update();
    float getYaw() const;
    float getPitch() const;
    float getRoll() const;
    float getYawRate() const;
    float getPitchRate() const;   
    float getRollRate() const;
    static void dmpDataReady();

private:
    IMUHandler(); 
    MPU6050 mpu;
    Quaternion q;           // Quaternion container
    VectorFloat gravity;    // Gravity vector
    float ypr[3];           // Yaw, Pitch, Roll angles
    float yprRate[3];       // Yaw, Pitch, Roll rates
    float lastYPR[3];       // Last Yaw, Pitch, Roll angles
    uint32_t lastTime;      // Last update time
    uint8_t fifoBuffer[64]; // FIFO storage buffer
    uint16_t packetSize;    // Expected DMP packet size
    uint8_t devStatus;      // Device status
    uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
    volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
    bool dmpReady;          // DMP initialization status
    bool calibrated;        // Calibration status
    float yawOffset, pitchOffset, rollOffset; // Calibration offsets
};

#endif // IMUHANDLER_H