#include <Arduino.h>
#include <FastAccelStepper.h>
#include <PID_v1.h>
#include "config.h"
#include "IMUHandler.h"

// Microstepping configuration (adjust based on your motor driver setting)
#define STEPS_PER_REV 200      // Full steps per revolution (typically 200 for 1.8° motors)
#define MICROSTEPPING 8        // Microstepping setting (e.g., 16, 32, etc.)

// Speed and acceleration
#define MAX_SPEED_RPM 100      // Maximum speed in RPM
#define ACCELERATION 2000      // Steps per second^2

// PID tuning parameters
float Kp = 2.0, Ki = 0.0, Kd = 0.0;

// PID variables for each motor
float input = 0, output1 = 0, output2 = 0, setpoint = 0; // Setpoint is the desired tilt angle (e.g., 0°)

// PID instances for each motor
PID pid1(&input, &output1, &setpoint, Kp, Ki, Kd, 1, DIRECT);
PID pid2(&input, &output2, &setpoint, Kp, Ki, Kd, 1, DIRECT);

// Stepper instances
FastAccelStepperEngine engine;
FastAccelStepper* stepper1 = nullptr;
FastAccelStepper* stepper2 = nullptr;

// Get IMU pitch angle
IMUHandler& imu = IMUHandler::getInstance();

void printAlignedValue(const char* label, float value, int width);
void processSerialCommands();
float calculateStepFrequency(float rpm);

void setup() {
    Serial.begin(115200);

    // Initialize IMU
    imu.initialize();
    imu.calibrate(true);

    // Initialize stepper engine
    engine.init();

    // Configure Motor 1
    stepper1 = engine.stepperConnectToPin(MOTOR1_STEP_PIN);
    if (!stepper1) {
        Serial.println("Motor 1 connection failed!");
        while (1);
    }
    stepper1->setDirectionPin(MOTOR1_DIR_PIN);
    stepper1->setEnablePin(MOTOR1_ENABLE_PIN);
    stepper1->setAutoEnable(true);
    stepper1->setSpeedInHz(calculateStepFrequency(MAX_SPEED_RPM));
    stepper1->setAcceleration(ACCELERATION);

    // Configure Motor 2
    stepper2 = engine.stepperConnectToPin(MOTOR2_STEP_PIN);
    if (!stepper2) {
        Serial.println("Motor 2 connection failed!");
        while (1);
    }
    stepper2->setDirectionPin(MOTOR2_DIR_PIN);
    stepper2->setEnablePin(MOTOR2_ENABLE_PIN);
    stepper2->setAutoEnable(true);
    stepper2->setSpeedInHz(calculateStepFrequency(MAX_SPEED_RPM));
    stepper2->setAcceleration(ACCELERATION);

    // Initialize PID controllers
    pid1.SetMode(AUTOMATIC);
    pid2.SetMode(AUTOMATIC);
    pid1.SetOutputLimits(-MAX_SPEED_RPM, MAX_SPEED_RPM);
    pid2.SetOutputLimits(-MAX_SPEED_RPM, MAX_SPEED_RPM);

    Serial.println("System Initialized. Use Serial to adjust PID parameters:");
    Serial.println("  Kp[value] (e.g., Kp2.5)");
    Serial.println("  Ki[value] (e.g., Ki0.5)");
    Serial.println("  Kd[value] (e.g., Kd1.0)");
    Serial.println("  setpoint[value] (e.g., setpoint0.0)");
}

void loop() {
    // Process Serial commands
    processSerialCommands();

    imu.update();
    input = imu.getPitch();

    // Update PID for both motors
    pid1.Compute();
    pid2.Compute();

    // Control Motor 1
    if (output1 > 0) {
        stepper1->runForward();
        stepper1->setSpeedInHz(calculateStepFrequency(output1));
    } else if (output1 < 0) {
        stepper1->runBackward();
        stepper1->setSpeedInHz(calculateStepFrequency(-output1));
    } else {
        stepper1->stopMove();
    }

    // Control Motor 2
    if (output2 > 0) {
        stepper2->runForward();
        stepper2->setSpeedInHz(calculateStepFrequency(output2));
    } else if (output2 < 0) {
        stepper2->runBackward();
        stepper2->setSpeedInHz(calculateStepFrequency(-output2));
    } else {
        stepper2->stopMove();
    }

    // Print data for plotting
    Serial.print("Input: ");
    Serial.print(input);
    Serial.print(", Output1: ");
    Serial.print(output1);
    Serial.print(", Output2: ");
    Serial.println(output2);

    Serial.print("Kp: ");
    Serial.print(pid1.GetKp());
    Serial.print(", Ki: ");
    Serial.print(pid1.GetKi());
    Serial.print(", Kd: ");
    Serial.println(pid1.GetKd());

    

    delay(100); // Loop frequency (adjust as needed for real-time performance)
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

// Function to calculate step frequency based on RPM
float calculateStepFrequency(float rpm) {
    return (rpm * STEPS_PER_REV * MICROSTEPPING) / 60.0;
}

// Function to process Serial commands for PID tuning
void processSerialCommands() {
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        command.trim();

        // Parse the command
        if (command.startsWith("Kp")) {
            Kp = command.substring(2).toFloat(); // Extract value after "Kp"
            pid1.SetTunings(Kp, Ki, Kd);
            pid2.SetTunings(Kp, Ki, Kd);
            Serial.print("Updated Kp: ");
            Serial.println(Kp);
        } else if (command.startsWith("Ki")) {
            Ki = command.substring(2).toFloat(); // Extract value after "Ki"
            pid1.SetTunings(Kp, Ki, Kd);
            pid2.SetTunings(Kp, Ki, Kd);
            Serial.print("Updated Ki: ");
            Serial.println(Ki);
        } else if (command.startsWith("Kd")) {
            Kd = command.substring(2).toFloat(); // Extract value after "Kd"
            pid1.SetTunings(Kp, Ki, Kd);
            pid2.SetTunings(Kp, Ki, Kd);
            Serial.print("Updated Kd: ");
            Serial.println(Kd);
        } else if (command.startsWith("setpoint")) {
            setpoint = command.substring(8).toFloat(); // Extract value after "setpoint"
            Serial.print("Updated Setpoint: ");
            Serial.println(setpoint);
        } else {
            Serial.println("Invalid command. Use:");
            Serial.println("  Kp[value] (e.g., Kp2.5)");
            Serial.println("  Ki[value] (e.g., Ki0.5)");
            Serial.println("  Kd[value] (e.g., Kd1.0)");
            Serial.println("  setpoint[value] (e.g., setpoint0.0)");
        }
    }
}