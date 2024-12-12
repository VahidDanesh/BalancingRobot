#include <Arduino.h>  
#include <FastAccelStepper.h>  
#include <PID_v1.h>  
#include <WiFi.h>  
#include <ESPAsyncWebServer.h>  
#include <AsyncTCP.h>  
#include <SPIFFS.h>  
#include "config.h"  
#include "IMUHandler.h"  



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
float speed1 = 0, speed2 = 0, filteredSpeed1 = 0, filteredSpeed2 = 0, filteredAvgSpeed = 0; // Motor speeds in Hz 
float alpha = 0.996;            // Complementary filter factor
float lastTime = 0;             // Last time for debugging


// LQR Gain Matrix (precomputed)  
float K[2][5] = {  
    {-847.4652, -116.2684, -10.0000, 0.0000, 0.0000},  
    {0.0000, 0.0000, 0.0000, 0.0000, 0.0000}  
};  


float speedFactor = 10.0f; // Speed scaling factor



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


    // Get current speed measurements in milliHz
    float lastSpeed1 = stepper1->getCurrentSpeedInMilliHz() / 1000;
    float lastSpeed2 = stepper2->getCurrentSpeedInMilliHz() / 1000;

    float lastAvgSpeed = (lastSpeed1 - lastSpeed2) / 2;


    // Map control inputs (torques) to motor speeds  
    speed1 = rpm2sps(speedFactor * ((u[0] + u[1]) / 2));  
    speed2 = rpm2sps(speedFactor * ((u[0] - u[1]) / 2));

    float avgSpeed = (speed1 + speed2) / 2;

    // Apply complementary filter
    // filteredSpeed1 = alpha * speed1 + (1 - alpha) * lastSpeed1;
    // filteredSpeed2 = alpha * speed2 + (1 - alpha) * lastSpeed2;  

    // filteredSpeed1 = constrain(filteredSpeed1, -MAX_SPEED, MAX_SPEED);
    // filteredSpeed2 = constrain(filteredSpeed2, -MAX_SPEED, MAX_SPEED);

    filteredAvgSpeed = alpha * avgSpeed + (1 - alpha) * lastAvgSpeed;

    filteredAvgSpeed = constrain(filteredAvgSpeed, -MAX_SPEED, MAX_SPEED);

    stepper1->setSpeedInHz(abs(filteredAvgSpeed));
    stepper2->setSpeedInHz(abs(filteredAvgSpeed));

    // Control Motor 1  
    if (filteredAvgSpeed > 0) { 
        stepper1->runForward();
        stepper2->runBackward();
    } else {   
        stepper1->runBackward();  
        stepper2->runForward();
    }  

    float nowTime = millis();
    float elapsedTime = nowTime - lastTime;

    if (elapsedTime > 1000) {


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
        Serial.println(filteredAvgSpeed);

        lastTime = nowTime;
    }

    if (abs(x[0] * RAD_TO_DEG) > 45) {
        stepper1->stopMove();
        stepper2->stopMove();
        Serial.println("Emergency Stop: Tilt angle exceeded safety limits!");
        return;
    }
    
    delay(20); // Loop frequency  
}  

void updateStateVector() {  
    // Update state vector from sensors  
    imu.update();  
    x[0] = imu.getRoll();       // Tilt angle (alpha)
    x[1] = imu.getRollRate();   // Tilt angular velocity (alpha_dot) 
    x[2] = getCurrentSpeedInMPS(stepper1, stepper2);                   // Forward velocity (v) - Placeholder  
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


    float speed = (mot1Speed - mot2Speed) / 2.0;


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

        if (command.startsWith("k")) {  
            speedFactor = command.substring(1).toFloat();  
  
            Serial.print("Updated speedFactor: ");  
            Serial.println(speedFactor);  
        }
        if (command.startsWith("a")) {  
            alpha = command.substring(1).toFloat();  
  
            Serial.print("Updated alpha: ");  
            Serial.println(alpha);  
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


