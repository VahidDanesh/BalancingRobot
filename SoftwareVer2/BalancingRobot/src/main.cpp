#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "IMUHandler.h"
#include <PID_v1.h>
#include <FastAccelStepper.h>

// IMU
IMUHandler& imu = IMUHandler::getInstance();

// PID variables
double input = 0, output = 0, setpoint = 0;
double kp = 2.0, ki = 5.0, kd = 1.0;
PID pid(&input, &output, &setpoint, kp, ki, kd, DIRECT);

// Stepper motor controllers
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper* motor1 = nullptr;
FastAccelStepper* motor2 = nullptr;

void setup() {
    Serial.begin(115200);

    // Initialize IMU
    Serial.println("Initializing IMU...");
    imu.initialize();
    imu.calibrate();

    // Initialize FastAccelStepper
    Serial.println("Initializing motors...");
    engine.init();

    // Motor 1 setup
    motor1 = engine.stepperConnectToPin(MOTOR1_STEP_PIN);
    if (motor1) {
        motor1->setDirectionPin(MOTOR1_DIR_PIN);
        motor1->setEnablePin(MOTOR1_ENABLE_PIN);
        motor1->setAutoEnable(true);
        motor1->setSpeedInHz(1000); // Default speed
        motor1->setAcceleration(1000); // Default acceleration
    } else {
        Serial.println("Failed to initialize Motor 1!");
    }

    // Motor 2 setup
    motor2 = engine.stepperConnectToPin(MOTOR2_STEP_PIN);
    if (motor2) {
        motor2->setDirectionPin(MOTOR2_DIR_PIN);
        motor2->setEnablePin(MOTOR2_ENABLE_PIN);
        motor2->setAutoEnable(true);
        motor2->setSpeedInHz(1000); // Default speed
        motor2->setAcceleration(1000); // Default acceleration
    } else {
        Serial.println("Failed to initialize Motor 2!");
    }

    // Set PID output limits
    pid.SetOutputLimits(-400, 400); // Limit to motor speed range
    pid.SetMode(AUTOMATIC);         // Enable the PID controller

    Serial.println("Setup complete!");
}

void loop() {
    // Update IMU
    imu.update();

    // Get pitch angle from IMU
    input = imu.getPitch();

    // Compute PID output
    pid.Compute();

    // Map PID output to motor speeds
    float motor1Speed = output;
    float motor2Speed = -output;

    // Set motor speeds and directions
    if (motor1) {
        motor1->setSpeedInHz(abs(motor1Speed));
        motor1->setDirection(motor1Speed > 0);
    }

    if (motor2) {
        motor2->setSpeedInHz(abs(motor2Speed));
        motor2->setDirection(motor2Speed > 0);
    }

    // Debugging
    Serial.print("Pitch: ");
    Serial.print(input);
    Serial.print(" Output: ");
    Serial.println(output);

    delay(10); // Run at 100Hz
}