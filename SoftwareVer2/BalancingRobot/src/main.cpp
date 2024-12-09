#include <Arduino.h>  
#include <FastAccelStepper.h>  
#include <PID_v1.h>  
#include <WiFi.h>  
#include <WebServer.h>  
#include <SPIFFS.h>  
#include "config.h"  
#include "IMUHandler.h"  

// Microstepping configuration  
#define STEPS_PER_REV 200  
#define MICROSTEPPING 32  

// Speed and acceleration  
#define MAX_SPEED_RPM 100  
#define ACCELERATION 8 * 200 * 100  

// Safety limits  
#define MAX_TILT_ANGLE 30.0f  
#define EMERGENCY_STOP_ANGLE 60.0f  

// Control modes  
#define PID_ANGLE 0  
#define PID_POS 1  
#define PID_SPEED 2  

uint8_t controlMode = PID_POS; // Default to angle+position control  

// PID tuning parameters  
float Kp_angle = 5.0, Ki_angle = 1.0, Kd_angle = 0.5;  
float Kp_pos = 2.0, Ki_pos = 0.5, Kd_pos = 0.1;  
float Kp_speed = 3.0, Ki_speed = 0.8, Kd_speed = 0.2;  

// PID variables  
float input_angle = 0, output_angle = 0, setpoint_angle = 0;  
float input_pos = 0, output_pos = 0, setpoint_pos = 0;  
float input_speed = 0, output_speed = 0, setpoint_speed = 0;  

// PID instances  
PID pid_angle(&input_angle, &output_angle, &setpoint_angle, Kp_angle, Ki_angle, Kd_angle, DIRECT);  
PID pid_pos(&input_pos, &output_pos, &setpoint_pos, Kp_pos, Ki_pos, Kd_pos, DIRECT);  
PID pid_speed(&input_speed, &output_speed, &setpoint_speed, Kp_speed, Ki_speed, Kd_speed, DIRECT);  

// Stepper instances  
FastAccelStepperEngine engine;  
FastAccelStepper* stepper1 = nullptr;  
FastAccelStepper* stepper2 = nullptr;  

// IMU instance  
IMUHandler& imu = IMUHandler::getInstance();  

// WiFi and Web Server  
WebServer server(WEB_SERVER_PORT);  

// Function declarations  
void setupWiFi();  
void setupWebServer();  
void handleWebRequests();  
void processSerialCommands();  
uint32_t calculateStepFrequency(uint32_t rpm);  
void updateControlMode();  

void setup() {  
    Serial.begin(115200);  

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

    // Configure Motor 1  
    stepper1 = engine.stepperConnectToPin(MOTOR1_STEP_PIN);  
    if (!stepper1) {  
        Serial.println("Motor 1 connection failed!");  
        while (1);  
    }  
    stepper1->setDirectionPin(MOTOR1_DIR_PIN);  
    stepper1->setEnablePin(MOTOR1_ENABLE_PIN);  
    stepper1->setAutoEnable(true);  
    stepper1->setSpeedInHz(0);  
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
    stepper2->setSpeedInHz(0);  
    stepper2->setAcceleration(ACCELERATION);  

    // Initialize PID controllers  
    pid_angle.SetMode(AUTOMATIC);  
    pid_pos.SetMode(AUTOMATIC);  
    pid_speed.SetMode(AUTOMATIC);  

    pid_angle.SetOutputLimits(-MAX_SPEED_RPM, MAX_SPEED_RPM);  
    pid_pos.SetOutputLimits(-MAX_TILT_ANGLE, MAX_TILT_ANGLE);  
    pid_speed.SetOutputLimits(-MAX_TILT_ANGLE, MAX_TILT_ANGLE);  

    // Setup WiFi and Web Server  
    setupWiFi();  
    setupWebServer();  

    Serial.println("System Initialized. Use Serial or Web Interface to adjust parameters.");  
}  

void loop() {  
    // Process Serial commands  
    processSerialCommands();  

    // Handle web requests  
    server.handleClient();  

    // Update IMU data  
    imu.update();  
    input_angle = imu.getRoll();  

    // Emergency stop if tilt angle exceeds safety limits  
    if (abs(input_angle) > EMERGENCY_STOP_ANGLE) {  
        stepper1->setSpeedInHz(0);  
        stepper2->setSpeedInHz(0);  
        Serial.println("Emergency Stop: Tilt angle exceeded safety limits!");  
        return;  
    }  

    // Update control mode  
    updateControlMode();  

    // Print data for debugging  
    Serial.print("Control Mode: ");  
    Serial.print(controlMode);  
    Serial.print(", Input Angle: ");  
    Serial.print(input_angle);  
    Serial.print(", Output Angle: ");  
    Serial.println(output_angle);  

    delay(10); // Adjust loop frequency as needed  
}  

void updateControlMode() {  
    switch (controlMode) {  
        case PID_ANGLE:  
            // Angle-only control  
            pid_angle.Compute();  
            stepper1->setSpeedInHz(calculateStepFrequency(abs(output_angle)));  
            stepper2->setSpeedInHz(calculateStepFrequency(abs(output_angle)));  
            break;  

        case PID_POS:  
            // Position control  
            pid_pos.Compute();  
            setpoint_angle = output_pos; // Position controller sets tilt angle  
            pid_angle.Compute();  
            stepper1->setSpeedInHz(calculateStepFrequency(abs(output_angle)));  
            stepper2->setSpeedInHz(calculateStepFrequency(abs(output_angle)));  
            break;  

        case PID_SPEED:  
            // Speed control  
            pid_speed.Compute();  
            setpoint_angle = output_speed; // Speed controller sets tilt angle  
            pid_angle.Compute();  
            stepper1->setSpeedInHz(calculateStepFrequency(abs(output_angle)));  
            stepper2->setSpeedInHz(calculateStepFrequency(abs(output_angle)));  
            break;  
    }  
}  

uint32_t calculateStepFrequency(uint32_t rpm) {  
    return (rpm * STEPS_PER_REV * MICROSTEPPING) / 60.0;  
}  

void processSerialCommands() {  
    if (Serial.available() > 0) {  
        String command = Serial.readStringUntil('\n');  
        command.trim();  

        if (command.startsWith("Kp_angle")) {  
            Kp_angle = command.substring(9).toFloat();  
            pid_angle.SetTunings(Kp_angle, Ki_angle, Kd_angle);  
            Serial.print("Updated Kp_angle: ");  
            Serial.println(Kp_angle);  
        } else if (command.startsWith("Ki_angle")) {  
            Ki_angle = command.substring(9).toFloat();  
            pid_angle.SetTunings(Kp_angle, Ki_angle, Kd_angle);  
            Serial.print("Updated Ki_angle: ");  
            Serial.println(Ki_angle);  
        } else if (command.startsWith("Kd_angle")) {  
            Kd_angle = command.substring(9).toFloat();  
            pid_angle.SetTunings(Kp_angle, Ki_angle, Kd_angle);  
            Serial.print("Updated Kd_angle: ");  
            Serial.println(Kd_angle);  
        } else if (command.startsWith("controlMode")) {  
            controlMode = command.substring(12).toInt();  
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
    server.on("/", HTTP_GET, []() {  
        File file = SPIFFS.open("/index.html", "r");  
        if (!file) {  
            server.send(500, "text/plain", "Failed to load index.html");  
            return;  
        }  
        server.streamFile(file, "text/html");  
        file.close();  
    });  

    // Handle other requests (e.g., commands, PID updates)  
    server.on("/command", HTTP_GET, []() {  
        String cmd = server.arg("cmd");  
        // Handle commands here  
        server.send(200, "text/plain", "Command received: " + cmd);  
    });  

    server.on("/pid", HTTP_POST, []() {  
        // Handle PID updates here  
        server.send(200, "text/plain", "PID values updated");  
    });  

    server.on("/mode", HTTP_GET, []() {  
        String mode = server.arg("set");  
        // Handle mode changes here  
        server.send(200, "text/plain", "Mode changed to: " + mode);  
    });  

    server.begin();  
    Serial.println("Web Server started.");  
}  