#include <Arduino.h>
#include <FastAccelStepper.h>
#include <PID_v1.h>
#include "config.h"
#include "IMUHandler.h"

// Microstepping configuration (adjust based on your motor driver setting)
#define STEPS_PER_REV 200      // Full steps per revolution (typically 200 for 1.8° motors)
#define MICROSTEPPING 2        // Microstepping setting (e.g., 16, 32, etc.)

// Speed and acceleration
#define MAX_SPEED_RPM 100     // Maximum speed in RPM
#define ACCELERATION 8*200*100      // Steps per second^2

// PID tuning parameters
float Kp = 2.0, Ki = 0.0, Kd = 0.0;

// PID variables for each motor
float input = 0, output1 = 0, output2 = 0, setpoint = 0; // Setpoint is the desired tilt angle (e.g., 0°)
uint32_t stepFrequency;

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
uint32_t calculateStepFrequency(uint32_t rpm);




void setup() {
    Serial.begin(115200);

    // Initialize the stepper engine  
    engine.init();  

    // Attach the stepper motor to the engine  
    stepper1 = engine.stepperConnectToPin(MOTOR1_STEP_PIN);  
    if (stepper1) {  
        // Configure the stepper motor  
        stepper1->setDirectionPin(MOTOR1_DIR_PIN);  
        stepper1->setEnablePin(MOTOR1_ENABLE_PIN, true); // Enable pin is active low  
        stepper1->setAutoEnable(true);                  // Automatically enable/disable motor  
        stepper1->setAcceleration(ACCELERATION);        // Set acceleration  
    } else {  
        Serial.println("Failed to initialize stepper motor!");  
        while (1); // Halt execution if stepper initialization fails  
    }  
}  



void loop() {  
    if (stepper1) {  
        // Calculate step frequency for the desired speed  
        stepFrequency = calculateStepFrequency(MAX_SPEED_RPM);  // Calculate step frequency for the desired speed


        // Run the motor in one direction for 5 seconds  
        stepper1->setSpeedInHz(stepFrequency); // Set speed in Hz  
        stepper1->runForward();                // Run forward  
        delay(2*5000);                           // Wait for 5 seconds  

        // Change direction and run for another 5 seconds  
        stepper1->runBackward();               // Run backward  
        delay(2*5000);                           // Wait for 5 seconds  
    }  
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
uint32_t calculateStepFrequency(uint32_t rpm) {
    return (rpm * STEPS_PER_REV * MICROSTEPPING) / 60.0 * 16;
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