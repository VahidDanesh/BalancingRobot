#include <Arduino.h>  
#include <FastAccelStepper.h>  
#include <PID_v1.h>  
#include <WiFi.h>  
#include <ESPAsyncWebServer.h>  
#include <AsyncTCP.h>  
#include <SPIFFS.h>  
#include "config.h"  
#include "IMUHandler.h"  




// LQR Gain Matrix (precomputed)  
float K[2][5] = {  
    {-75.0848, -3.3575, 3.1623, 0.0000, 0.0000},  
    {0.0000, 0.0000, 0.0000, 0.0000, 0.0000}  
};  


float speedFactor = 600.0f; // Speed scaling factor
// Stepper instances  
FastAccelStepperEngine engine;  
FastAccelStepper* stepper1 = nullptr;  
FastAccelStepper* stepper2 = nullptr;  

// IMU instance  
IMUHandler& imu = IMUHandler::getInstance();  

// State vector  
float x[5] = {0, 0, 0, 0, 0}; // [alpha, alpha_dot, v, theta, theta_dot]
float wr[5] = {0, 0, 0, 0, 0}; // Reference state vector  
float u[2] = {0, 0};          // Control inputs [u1, u2] 
float speed1 = 0, speed2 = 0; // Motor speeds in Hz 

// Function prototypes   
void setupWiFi();  
void setupWebServer();  
void handleWebRequests();  
void processSerialCommands();  
float rpm2sps(float rpm);
void initMotor(FastAccelStepper* stepper, uint8_t stepPin, uint8_t dirPin, uint8_t enablePin);  
void updateControlMode();  
void updateStateVector();
float getCurrentSpeedInMPS(FastAccelStepper* stepMot1, FastAccelStepper* stepMot2);

void setup() {  
    Serial.begin(SERIAL_BAUD);


    // Initialize SPIFFS  
    if (!SPIFFS.begin(true)) {  
        Serial.println("Failed to mount SPIFFS!");  
        while (1);  
    }    

    // Initialize IMU  
    imu.initialize();  
    imu.calibrate(true);  

    // Initialize stepper engine  
    engine.init();  
    stepper1 = engine.stepperConnectToPin(MOTOR1_STEP_PIN);  
    if (!stepper1) {  
        Serial.println("Motor connection failed!");  
        while (1);  
    }  
    stepper1->setDirectionPin(MOTOR1_DIR_PIN);  
    stepper1->setEnablePin(MOTOR1_ENABLE_PIN);  
    stepper1->setAutoEnable(true);  
    stepper1->setSpeedInHz(1);  
    stepper1->setAcceleration(MAX_ACCELERATION);  
    Serial.println("Motor Initialized");

    stepper2 = engine.stepperConnectToPin(MOTOR2_STEP_PIN);
    if (!stepper2) {
        Serial.println("Motor connection failed!");
        while (1);
    }
    stepper2->setDirectionPin(MOTOR2_DIR_PIN);
    stepper2->setEnablePin(MOTOR2_ENABLE_PIN);
    stepper2->setAutoEnable(true);
    stepper2->setSpeedInHz(1);
    stepper2->setAcceleration(MAX_ACCELERATION);
    Serial.println("Motor Initialized");


    Serial.println("System Initialized. LQR control is active.");  
}  

void loop() {  
    // Process Serial commands  
    processSerialCommands(); 

    // Update state vector  
    updateStateVector();


    // Compute control inputs using LQR  
    u[0] = 0; // Reset control input u1  
    u[1] = 0; // Reset control input u2  
    for (int i = 0; i < 5; i++) {  
        u[0] += -K[0][i] * (x[i] - wr[i]);  
        u[1] += -K[1][i] * (x[i] - wr[i]);  
    }  



    // Map control inputs (torques) to motor speeds  
    speed1 = speedFactor * rpm2sps((u[0] + u[1]) / 2);  
    speed2 = speedFactor * rpm2sps((u[0] - u[1]) / 2);
    

    speed1 = constrain(speed1, -MAX_SPEED_RPM, MAX_SPEED_RPM);
    speed2 = constrain(speed2, -MAX_SPEED_RPM, MAX_SPEED_RPM);


    // Control Motor 1  
    if (speed1 > 0) {  
        stepper1->setSpeedInHz(speed1);  
        stepper1->runForward();
    } else {  
        stepper1->setSpeedInHz(speed1);  
        stepper1->runBackward();  
    }  

    // Control Motor 2  
    if (speed2 > 0) {  
        stepper2->setSpeedInHz(speed2);  
        stepper2->runForward();  
    } else {  
        stepper2->setSpeedInHz(speed2);  
        stepper2->runBackward();  
    }  

    // Print state and control inputs for debugging  
    Serial.print("State: ");  
    for (int i = 0; i < 5; i++) {  
        Serial.print(x[i]);  
        Serial.print(" ");  
    }  
    // Print data for plotting
    Serial.print("Input: ");
    Serial.print(x[0]);
    Serial.print(", Output1: ");
    Serial.print(speed1);
    Serial.print(", Output2: ");
    Serial.println(speed2);

    delay(10); // Loop frequency  
}  

void updateStateVector() {  
    // Update state vector from sensors  
    imu.update();  
    x[0] = imu.getRoll();       // Tilt angle (alpha)
    x[1] = imu.getRollRate();   // Tilt angular velocity (alpha_dot) 
    x[2] = 0;                   // Forward velocity (v) - Placeholder  
    x[3] = imu.getYaw();        // Heading angle (theta)  
    x[4] = imu.getYawRate();    // Heading angular velocity (theta_dot) 

}  

float rpm2sps(float speedRPM) {  
    return (speedRPM * STEPS_PER_REV * MICROSTEPS) / 60.0;  
}  



float getCurrentSpeedInMPS(FastAccelStepper* stepper1, FastAccelStepper* stepper2) {  

    // Check if the stepper pointers are valid  
    if (stepper1 == nullptr || stepper2 == nullptr) {  
        Serial.println("Error: Stepper motor not initialized!");  
        return 0.0; 
    } 

    int32_t mot1Speed = stepper1->getCurrentSpeedInMilliHz();
    int32_t mot2Speed = stepper2->getCurrentSpeedInMilliHz();


    float speed = (mot1Speed + mot2Speed) / 2.0;


    // convert the speed of stepper in us to m/s using wheelRadius
    float speedInHZ = speed / 1000.0;
    float speedInRPS = speedInHZ  / (STEPS_PER_REV * MICROSTEPS);
    float speedInMPS = speedInRPS * TWO_PI *  WHEEL_RADIUS;


    return speedInMPS; 
}


void initMotor(FastAccelStepper* stepper, uint8_t stepPin, uint8_t dirPin, uint8_t enablePin) {  
    stepper = engine.stepperConnectToPin(stepPin);  
    if (!stepper) {  
        Serial.println("Motor connection failed!");  
        while (1);  
    }  
    stepper->setDirectionPin(dirPin);  
    stepper->setEnablePin(enablePin);  
    stepper->setAutoEnable(true);  
    stepper->setSpeedInHz(0);  
    stepper->setAcceleration(MAX_ACCELERATION);  
    Serial.println("Motor Initialized");
}

void processSerialCommands() {  
    if (Serial.available() > 0) {  
        String command = Serial.readStringUntil('\n');  
        command.trim();  

        if (command.startsWith("Kp")) {  
            speedFactor = command.substring(2).toFloat();  
  
            Serial.print("Updated speedFactor: ");  
            Serial.println(speedFactor);  
        }

    }  
}  

void setupWiFi() {  
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);  
    while (WiFi.status() != WL_CONNECTED) {  
        delay(1000);  
        Serial.println("Connecting to WiFi...");  
    }  
    Serial.println("Connected to WiFi!");  
    Serial.print("IP Address: ");  
    Serial.println(WiFi.localIP());  
}  


