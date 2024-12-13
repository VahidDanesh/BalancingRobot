#include <Arduino.h>  
#include <FastAccelStepper.h>  
#include <PID_v1.h>  
#include <WiFi.h>  
#include <ESPAsyncWebServer.h>  
#include <AsyncTCP.h>  
#include <SPIFFS.h>  
#include "config.h"  
#include "IMUHandler.h"  



uint8_t controlMode = PID_ANGLE; // Default to angle+position control  

// PID tuning parameters  
float Kp_angle = 5.0, Ki_angle = 0.0, Kd_angle = 0.3;  
float Kp_pos = 2.0, Ki_pos = 0.5, Kd_pos = 0.1;  
float Kp_speed = 3.0, Ki_speed = 0.8, Kd_speed = 0.2;  

// PID variables  
float input_angle = 0, output_angle = 0, setpoint_angle = 0;  
float input_pos = 0, output_pos = 0, setpoint_pos = 0;  
float input_speed = 0, output_speed = 0, setpoint_speed = 0;  

// PID instances  
PID pid_angle(&input_angle, &output_angle, &setpoint_angle, Kp_angle, Ki_angle, Kd_angle, 1, REVERSE); 
PID pid_pos(&input_pos, &output_pos, &setpoint_pos, Kp_pos, Ki_pos, Kd_pos, 1, REVERSE);  
PID pid_speed(&input_speed, &output_speed, &setpoint_speed, Kp_speed, Ki_speed, Kd_speed, 1, REVERSE);  

// Stepper instances  
FastAccelStepperEngine engine;  
FastAccelStepper* stepper1 = nullptr;  
FastAccelStepper* stepper2 = nullptr;  

// IMU instance  
IMUHandler& imu = IMUHandler::getInstance();  

// WiFi and Web Server  
AsyncWebServer server(WEB_SERVER_PORT);  

// Function declarations  
void setupWiFi();  
void setupWebServer();  
void handleWebRequests();  
void processSerialCommands();  
float rpm2sps(float rpm);
void updateControlMode();  

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

    // Initialize motors  
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



    // Initialize PID controllers  
    pid_angle.SetMode(AUTOMATIC);  
    pid_pos.SetMode(AUTOMATIC);  
    pid_speed.SetMode(AUTOMATIC);  

    pid_angle.SetOutputLimits(-MAX_SPEED_RPM, MAX_SPEED_RPM);  
    pid_pos.SetOutputLimits(-MAX_TILT_ANGLE, MAX_TILT_ANGLE);  
    pid_speed.SetOutputLimits(-MAX_TILT_ANGLE, MAX_TILT_ANGLE);  

    pid_angle.SetSampleTime(5);  // 200 Hz 
    pid_pos.SetSampleTime(10);  // 100 Hz
    pid_speed.SetSampleTime(10);  // 100 Hz

    // Setup WiFi and Web Server  
    setupWiFi();  
    setupWebServer();  

    Serial.println("System Initialized. Use Serial or Web Interface to adjust parameters.");  
}  

void loop() {  
    // Process Serial commands  
    processSerialCommands();  
 

    // Update IMU data  
    imu.update();  
    input_angle = imu.getRoll() * RAD_TO_DEG;  

    // Emergency stop if tilt angle exceeds safety limits  
    if (abs(input_angle) > EMERGENCY_STOP_ANGLE) {  
        stepper1->setSpeedInHz(0);
        stepper2->setSpeedInHz(0);
        Serial.println("Emergency Stop: Tilt angle exceeded safety limits!");  
    }  

    // Update control mode  
    updateControlMode();  

    // Print data for debugging  
    Serial.print("Control Mode: ");  
    Serial.print(controlMode);  
    Serial.print(", Input: ");  
    Serial.print(input_angle);  
    Serial.print(", Output: ");  
    Serial.println(output_angle);  

    delay(10); // Adjust loop frequency as needed  
}  

void updateControlMode() {
    switch (controlMode) {
        case PID_ANGLE:
            // Angle-only control
            pid_angle.Compute();
            output_angle = constrain(output_angle, -MAX_SPEED_RPM, MAX_SPEED_RPM);
            stepper1->setSpeedInHz(rpm2sps(abs(output_angle)));
            stepper2->setSpeedInHz(rpm2sps(abs(output_angle)));
            if (output_angle > 0) {
                stepper1->runForward();
                stepper2->runBackward();
            } else {
                stepper1->runBackward();
                stepper2->runForward();
            }
            break;

        case PID_POS:
            // Position control
            pid_speed.Compute();
            setpoint_pos = output_speed;  // Speed controller sets position target
            pid_pos.Compute();
            setpoint_angle = output_pos; // Position controller sets angle target
            pid_angle.Compute();
            output_angle = constrain(output_angle, -MAX_SPEED_RPM, MAX_SPEED_RPM);
            stepper1->setSpeedInHz(rpm2sps(abs(output_angle)));
            stepper2->setSpeedInHz(rpm2sps(abs(output_angle)));
            if (output_angle > 0) {
                stepper1->runForward();
                stepper2->runBackward();
            } else {
                stepper1->runBackward();
                stepper2->runForward();
            }
            break;

        case PID_SPEED:
            // Speed control
            pid_speed.Compute();
            setpoint_angle = output_speed;  // Speed controller sets angle target
            pid_angle.Compute();
            output_angle = constrain(output_angle, -MAX_SPEED_RPM, MAX_SPEED_RPM);
            stepper1->setSpeedInHz(rpm2sps(abs(output_angle)));
            stepper2->setSpeedInHz(rpm2sps(abs(output_angle)));
            if (output_angle > 0) {
                stepper1->runForward();
                stepper2->runBackward();
            } else {
                stepper1->runBackward();
                stepper2->runForward();
            }
            break;
    }
}
float rpm2sps(float rpm) {  
    return (rpm * STEPS_PER_REV * MICROSTEPS) / 60.0;  
}  


void processSerialCommands() {  
    if (Serial.available() > 0) {  
        String command = Serial.readStringUntil('\n');  
        command.trim();  

        if (command.startsWith("kp")) {  
            Kp_angle = command.substring(2).toFloat();  
            pid_angle.SetTunings(Kp_angle, Ki_angle, Kd_angle);  
            Serial.print("Updated Kp_angle: ");  
            Serial.println(Kp_angle);  
        } else if (command.startsWith("ki")) {  
            Ki_angle = command.substring(2).toFloat();  
            pid_angle.SetTunings(Kp_angle, Ki_angle, Kd_angle);  
            Serial.print("Updated Ki_angle: ");  
            Serial.println(Ki_angle);  
        } else if (command.startsWith("kd")) {  
            Kd_angle = command.substring(2).toFloat();  
            pid_angle.SetTunings(Kp_angle, Ki_angle, Kd_angle);  
            Serial.print("Updated Kd_angle: ");  
            Serial.println(Kd_angle);  
        } else if (command.startsWith("c")) {  
            controlMode = command.substring(1).toInt();  
            Serial.print("Updated Control Mode: ");  
            Serial.println(controlMode);  
        } else {  
            Serial.println("Invalid command.");  
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


void setupWebServer() {  
    // Serve the index.html file  
    server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {  
        request->send(SPIFFS, "/index.html", "text/html");  
    });  

    server.on("/pid", HTTP_POST, [](AsyncWebServerRequest *request) {
        if (!request->hasParam("type", true)) {
            request->send(400, "text/plain", "Missing PID type parameter");
            return;
        }

        String pidType = request->getParam("type", true)->value();
        float Kp, Ki, Kd;

        if (pidType == "angle") {
            Kp = request->getParam("Kp_angle", true)->value().toFloat();
            Ki = request->getParam("Ki_angle", true)->value().toFloat();
            Kd = request->getParam("Kd_angle", true)->value().toFloat();
            pid_angle.SetTunings(Kp, Ki, Kd);
        } else if (pidType == "pos") {
            Kp = request->getParam("Kp_pos", true)->value().toFloat();
            Ki = request->getParam("Ki_pos", true)->value().toFloat();
            Kd = request->getParam("Kd_pos", true)->value().toFloat();
            pid_pos.SetTunings(Kp, Ki, Kd);
        } else if (pidType == "speed") {
            Kp = request->getParam("Kp_speed", true)->value().toFloat();
            Ki = request->getParam("Ki_speed", true)->value().toFloat();
            Kd = request->getParam("Kd_speed", true)->value().toFloat();
            pid_speed.SetTunings(Kp, Ki, Kd);
        } else {
            request->send(400, "text/plain", "Invalid PID type");
            return;
        }

        request->send(200, "text/plain", "PID values updated successfully");
    });

    // Start the server  
    server.begin();  
    Serial.println("Web Server started.");  
}  