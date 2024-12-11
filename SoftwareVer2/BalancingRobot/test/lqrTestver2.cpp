#include <Arduino.h>  
#include <FastAccelStepper.h>  
#include <PID_v1.h>  
#include <WiFi.h>  
#include <ESPAsyncWebServer.h>  
#include <AsyncTCP.h>  
#include <SPIFFS.h>  
#include "config.h"  
#include "IMUHandler.h"  

// this program has some problem, first of all the motors start screaming when they reach to mas speed
// second the LQR is not working properly, it is not able to reach the target position, maybe tuning the scaleFactor will help
// third, the control input here assumed to be motor acceleration, I'm not sure if this aporoach is correct or not



// Stepper instances  
FastAccelStepperEngine engine;  
FastAccelStepper* stepper1 = nullptr;  
FastAccelStepper* stepper2 = nullptr;  

// IMU instance  
IMUHandler& imu = IMUHandler::getInstance();  


// LQR Gain Matrix (precomputed)  
float K[2][5] = {  
    {-243.8678, -8.2356, 10.0000, 0.0000, 0.0000},  
    {0.0000, 0.0000, 0.0000, 0.0000, 0.0000}  
};  



// State vector  
float x[5] = {0, 0, 0, 0, 0}; // [alpha, alpha_dot, v, theta, theta_dot]
float wr[5] = {0, 0, 0, 0, 0}; // Reference state vector  
float u[2] = {0, 0};          // Control inputs [u1, u2] 
float speed1 = 0, speed2 = 0; // Motor speeds in Hz
float scaleFactor = 1.0f; // Speed scaling factor
float acc1 = 0, acc2 = 0; // Acceleration values for motors
bool setSpeed = false; // Flag to set speed only once


// Function prototypes   
void setupWiFi();  
void setupWebServer();  
void handleWebRequests();  
void processSerialCommands();  
float rpm2sps(float rpm);
float rpss2spss(float accRPSS);
void initMotor(FastAccelStepper* stepper, uint8_t stepPin, uint8_t dirPin, uint8_t enablePin);  
void updateControlMode();  
void updateStateVector();
float getCurrentSpeedInMPS(FastAccelStepper* stepMot1, FastAccelStepper* stepMot2);
float constraint(float value, float min, float max);

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
    stepper1->setSpeedInHz(0);  
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
    stepper2->setSpeedInHz(0);
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

    acc1 = (u[0] + u[1]) / 2; // Acceleration for motor 1 [rad/s^2]
    acc2 = (u[0] - u[1]) / 2; // Acceleration for motor 2 [rad/s^2]

    acc1 = scaleFactor * rpss2spss(acc1); // Convert to steps/s^2
    acc2 = scaleFactor * rpss2spss(acc2); // Convert to steps/s^2

    acc1 = constraint(acc1, -MAX_ACCELERATION, MAX_ACCELERATION);
    acc2 = constraint(acc2, -MAX_ACCELERATION, MAX_ACCELERATION);




    // // Map control inputs (torques) to motor speeds  
    // speed1 = scaleFactor * rpm2sps((u[0] + u[1]) / 2);  
    // speed2 = scaleFactor * rpm2sps((u[0] - u[1]) / 2);

    // speed1 = constraint(speed1, -rpm2sps(MAX_SPEED_RPM), rpm2sps(MAX_SPEED_RPM)); 
    // speed2 = constraint(speed2, -rpm2sps(MAX_SPEED_RPM), rpm2sps(MAX_SPEED_RPM));


    // // Control Motor 1  
    // if (acc1 > 0) {  
    //     stepper1->setAcceleration(abs(acc1));  
    //     stepper1->runForward(); 
    // } else {  
    //     stepper1->setAcceleration(abs(acc1));  
    //     stepper1->runBackward();  
    // }  

    // // Control Motor 2  
    // if (acc2 > 0) {  
    //     stepper2->setAcceleration(abs(acc2));  
    //     stepper2->runBackward();  
    // } else {  
    //     stepper2->setAcceleration(abs(acc2));  
    //     stepper2->runForward();  
    // }  
    if (!setSpeed) {
        stepper1->setSpeedInHz(MAX_SPEED);
        stepper2->setSpeedInHz(MAX_SPEED);
        setSpeed = true;
    } 

    // Control Motor 1 using moveByAcceleration()
    stepper1->moveByAcceleration(acc1);

    // Control Motor 2 using moveByAcceleration()
    stepper2->moveByAcceleration(-acc2);

    if (abs(x[0] * RAD_TO_DEG) > MAX_TILT_ANGLE) {  
        Serial.println("Emergency Stop: Tilt angle exceeded!");  
        stepper1->stopMove();  
        stepper2->stopMove();  
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
    Serial.print(acc1);
    Serial.print(", Output2: ");
    Serial.println(acc2);

    delay(100); // Loop frequency  
}  

void updateStateVector() {  
    // Update state vector from sensors  
    imu.update();  
    x[0] = imu.getRoll();       // Tilt angle (alpha)
    x[1] = imu.getRollRate();   // Tilt angular velocity (alpha_dot) 
    x[2] = getCurrentSpeedInMPS(stepper1, stepper2);                   // Forward velocity (v) m/s - Placeholder  
    x[3] = imu.getYaw();        // Heading angle (theta)  
    x[4] = imu.getYawRate();    // Heading angular velocity (theta_dot) 
}  

float rpm2sps(float speedRPM) {  
    return (speedRPM * STEPS_PER_REV * MICROSTEPS) / 60.0;  
}  

float rpss2spss (float accRPSS) {  
    return (accRPSS * STEPS_PER_REV * MICROSTEPS) / TWO_PI;  
}


float constraint(float value, float min, float max) {  
    if (value < min) {  
        return min;  
    } else if (value > max) {  
        return max;  
    } else {  
        return value;  
    }  
}

float getCurrentSpeedInMPS(FastAccelStepper* stepper1, FastAccelStepper* stepper2) {  

    // Check if the stepper pointers are valid  
    if (stepper1 == nullptr || stepper2 == nullptr) {  
        Serial.println("Error: Stepper motor not initialized!");  
        return 0.0; 
    } 

    int32_t mot1Speed = stepper1->getCurrentSpeedInMilliHz(false);
    int32_t mot2Speed = stepper2->getCurrentSpeedInMilliHz(false);


    float speed = abs((mot1Speed - mot2Speed) / 2.0);


    // convert the speed of stepper in us to m/s using wheelRadius
    float speedInHZ = speed / 1000.0;
    float speedInRPS = speedInHZ  / (STEPS_PER_REV * MICROSTEPS);
    float speedInMPS = speedInRPS * TWO_PI *  WHEEL_RADIUS;


    return speedInMPS; 
}




void processSerialCommands() {  
    if (Serial.available() > 0) {  
        String command = Serial.readStringUntil('\n');  
        command.trim();  

        if (command.startsWith("Kp")) {  
            scaleFactor = command.substring(2).toFloat();  
  
            Serial.print("Updated scaleFactor: ");  
            Serial.println(scaleFactor);  
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


