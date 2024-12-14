#include <Arduino.h>  
#include <FastAccelStepper.h>  
#include <PID_v1.h>  
#include <WiFi.h>  
#include <ESPmDNS.h>
#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>  
#include <AsyncTCP.h>  
#include <SPIFFS.h>  
#include "config.h"  
#include "IMUHandler.h"  



uint8_t controlMode = PID_ANGLE; // Default to angle+position control  
unsigned long previousMillis = 0;
const unsigned long interval = 1000;

// PID tuning parameters  
float Kp_angle = 5.0, Ki_angle = 0.0, Kd_angle = 0.3;  
float Kp_pos = 2.0, Ki_pos = 0.5, Kd_pos = 0.1;  
float Kp_speed = 3.0, Ki_speed = 0.8, Kd_speed = 0.2;  

// PID variables  
float input_angle = 0, output_angle = 0, setpoint_angle = 0;  
float input_pos = 0, output_pos = 0, setpoint_pos = 0;  
float input_speed = 0, output_speed = 0, setpoint_speed = 0;  

float avgSpeedInput = 0, stepperLSpeed = 0, stepperRSpeed = 0, steer = 0;

// PID instances  
PID pid_angle(&input_angle, &output_angle, &setpoint_angle, Kp_angle, Ki_angle, Kd_angle, 1, REVERSE); 
PID pid_pos(&input_pos, &output_pos, &setpoint_pos, Kp_pos, Ki_pos, Kd_pos, 1, REVERSE);  
PID pid_speed(&input_speed, &output_speed, &setpoint_speed, Kp_speed, Ki_speed, Kd_speed, 1, REVERSE);  

// Stepper instances  
FastAccelStepperEngine engine;  
FastAccelStepper* stepperL = nullptr;  
FastAccelStepper* stepperR = nullptr;  

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
float getRobotPos(FastAccelStepper* stepperL, FastAccelStepper* stepperR);
float getRobotSpeed(FastAccelStepper* stepperL, FastAccelStepper* stepperR);
void setRobotSpeed(float stepperLSpeed, float stepperRSpeed);
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
    stepperL = engine.stepperConnectToPin(MOTORL_STEP_PIN);  
    if (!stepperL) {  
        Serial.println("Motor connection failed!");  
        while (1);  
    }  
    stepperL->setDirectionPin(MOTORL_DIR_PIN);  
    stepperL->setEnablePin(MOTORL_ENABLE_PIN);  
    stepperL->setAutoEnable(true);  
    stepperL->setSpeedInHz(1);  
    stepperL-> setCurrentPosition(0);
    stepperL->setAcceleration(MAX_ACCELERATION);  
    Serial.println("Motor Initialized");

    stepperR = engine.stepperConnectToPin(MOTORR_STEP_PIN);
    if (!stepperR) {
        Serial.println("Motor connection failed!");
        while (1);
    }
    stepperR->setDirectionPin(MOTORR_DIR_PIN);
    stepperR->setEnablePin(MOTORR_ENABLE_PIN);
    stepperR->setAutoEnable(true);
    stepperR->setSpeedInHz(1);
    stepperR->setCurrentPosition(0);
    stepperR->setAcceleration(MAX_ACCELERATION);
    Serial.println("Motor Initialized");



    // Initialize PID controllers  
    pid_angle.SetMode(AUTOMATIC);  
    pid_pos.SetMode(AUTOMATIC);  
    pid_speed.SetMode(AUTOMATIC);  

    pid_angle.SetOutputLimits(-MAX_SPEED_RPM, MAX_SPEED_RPM);  
    // pid_pos.SetOutputLimits(-MAX_TILT_ANGLE, MAX_TILT_ANGLE);  
    // pid_speed.SetOutputLimits(-MAX_TILT_ANGLE, MAX_TILT_ANGLE);  

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
    input_angle = imu.getRoll();
    input_speed = getRobotSpeed(stepperL, stepperR);
        // Update control mode  
    updateControlMode(); 
    stepperLSpeed = output_angle + steer;  // speed in rpm
    stepperRSpeed = output_angle - steer;  // speed in rpm
    setRobotSpeed(stepperLSpeed, stepperRSpeed);  



    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        // Emergency stop if tilt angle exceeds safety limits  
        if (abs(input_angle) > EMERGENCY_STOP_ANGLE) {  
            stepperLSpeed = 1;
            stepperRSpeed = 1;
            Serial.println("Emergency Stop: Tilt angle exceeded safety limits!");  
        }  
        if (WiFi.status() != WL_CONNECTED) {
            Serial.print("Reconnecting to WiFi...");
            Serial.println(WIFI_SSID);
            WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
        }
    }

    
    // // Print data for debugging  
    // Serial.print("Control Mode: ");  
    // Serial.print(controlMode);  
    // Serial.print(", Input: ");  
    // Serial.print(input_angle);  
    // Serial.print(", Output: ");  
    // Serial.println(output_angle);  

    // delay(10); // Adjust loop frequency as needed  
}  

void updateControlMode() {
    switch (controlMode) {
        case PID_ANGLE:
            // Angle-only control
            setpoint_angle = avgSpeedInput;  // Speed controller sets angle target
            pid_angle.Compute();
            break;

        case PID_POS:
            // Position control
            pid_speed.Compute();
            setpoint_pos = avgSpeedInput; //output_speed;  // Speed controller sets position target
            input_pos = getRobotPos(stepperL, stepperR);
            

            pid_pos.Compute();
            setpoint_angle = output_pos; // Position controller sets angle target
            pid_angle.Compute();
            break;

        case PID_SPEED:
            // Speed control
            setpoint_speed = avgSpeedInput;  // Speed controller sets speed target
            pid_speed.Compute();
            setpoint_angle = output_speed;  // Speed controller sets angle target
            pid_angle.Compute();
            break;
    }
}
float rpm2sps(float rpm) {  
    return (rpm * STEPS_PER_REV * MICROSTEPS) / 60.0;  
}  


float getRobotPos(FastAccelStepper* stepperL, FastAccelStepper* stepperR) {  
    float motPos =  (stepperL->getCurrentPosition() - stepperR->getCurrentPosition()) / 2.0;
    float pos = motPos / (STEPS_PER_REV * MICROSTEPS) * TWO_PI * WHEEL_RADIUS;
    return pos;  
}

float getRobotSpeed(FastAccelStepper* stepperL, FastAccelStepper* stepperR) {  
    // Check if the stepper pointers are valid  
    if (stepperL == nullptr || stepperR == nullptr) {  
        Serial.println("Error: Stepper motor not initialized!");  
        return 0.0; 
    } 

    int32_t mot1Speed = stepperL->getCurrentSpeedInMilliHz();
    int32_t mot2Speed = stepperR->getCurrentSpeedInMilliHz();


    float speed = (mot1Speed - mot2Speed) / 2.0;


    // convert the speed of stepper in us to m/s using wheelRadius
    float speedInHZ = speed / 1000.0;
    float speedInRPS = speedInHZ  / (STEPS_PER_REV * MICROSTEPS);
    float speedInMPS = speedInRPS * TWO_PI *  WHEEL_RADIUS;


    return speedInMPS;  
}

void setRobotSpeed(float stepperLSpeed, float stepperRSpeed) {  
    stepperLSpeed = (rpm2sps(stepperLSpeed));
    stepperRSpeed = (rpm2sps(stepperRSpeed));

    if (!stepperL->setSpeedInHz(abs(stepperLSpeed))) {
        stepperL->stopMove();
    } else if (!stepperR->setSpeedInHz(abs(stepperRSpeed))) {
        stepperR->stopMove();
    }

    if (stepperLSpeed > 0){
        stepperL->runForward();
    } else {
        stepperL->runBackward();
    }
    if (stepperRSpeed > 0){
        stepperR->runBackward();
    } else {
        stepperR->runForward();
    }

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
    Serial.print("Connected to WiFi! ");
    Serial.println(WIFI_SSID);

    if (MDNS.begin("robot")) {
        Serial.println("Hostname set to: http://robot.local");
    } else {
        Serial.println("Error setting up MDNS responder!");
    }

    Serial.print("IP Address: ");  
    Serial.println(WiFi.localIP());  
}  


void setupWebServer() {  
    // Serve the index.html file  
    server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {  
        request->send(SPIFFS, "/index.html", "text/html");  
    });  

    // Handle PID updates
    server.on("/pid", HTTP_POST, [](AsyncWebServerRequest* request) {
        // // Check if the request has a JSON body
        // if (request->contentType() == "application/json") {
        //     // Parse the JSON body
        //     StaticJsonDocument<256> doc;
        //     String json = "";
        //     if (request->hasParam("plain", true)) {
        //         json = request->getParam("plain", true)->value();
        //         Serial.println("Received JSON: " + json);

        //         DeserializationError error = deserializeJson(doc, json);

        //         if (error) {
        //             request->send(400, "text/plain", "Invalid JSON");
        //             Serial.println("Invalid JSON received" + String(error.c_str()));
        //             return;
        //     }}

  
            // // Extract the PID type and values
            // String pidType = doc["type"];
            // float Kp = doc[String("Kp_") + pidType];
            // float Ki = doc[String("Ki_") + pidType];
            // float Kd = doc[String("Kd_") + pidType];
            
            // Serial.println(pidType);
            // Serial.println(Kp);
            // Serial.println(Ki);
            // Serial.println(Kd);

            // for (JsonPair kv : doc.as<JsonObject>()) {
            //     Serial.print("Key: ");
            //     Serial.print(kv.key().c_str());
            //     Serial.print(", Value: ");
            //     Serial.println(kv.value().as<String>());
            // }
        if (request->hasParam("type")) {
            String pidType = request->getParam("type")->value();
            float Kp = request->getParam(String("Kp_") + pidType)->value().toFloat();
            float Ki = request->getParam(String("Ki_") + pidType)->value().toFloat();
            float Kd = request->getParam(String("Kd_") + pidType)->value().toFloat();

            // Update the corresponding PID controller
            if (pidType == "angle") {
                pid_angle.SetTunings(Kp, Ki, Kd);
                Serial.printf("Updated Angle PID: Kp=%.2f, Ki=%.2f, Kd=%.2f\n", Kp, Ki, Kd);
            } else if (pidType == "pos") {
                pid_pos.SetTunings(Kp, Ki, Kd);
                Serial.printf("Updated Position PID: Kp=%.2f, Ki=%.2f, Kd=%.2f\n", Kp, Ki, Kd);
            } else if (pidType == "speed") {
                pid_speed.SetTunings(Kp, Ki, Kd);
                Serial.printf("Updated Speed PID: Kp=%.2f, Ki=%.2f, Kd=%.2f\n", Kp, Ki, Kd);
            } else {
                request->send(400, "text/plain", "Invalid PID type");
                Serial.println("Invalid PID type");
                return;
            }

            request->send(200, "text/plain", "PID values updated successfully");
            Serial.println("PID values updated successfully");
        } else {
            request->send(400, "text/plain", "Invalid content type");
            Serial.println("Invalid content type");
        }
    });


    // Handle commands (e.g., movement)
    server.on("/command", HTTP_GET, [](AsyncWebServerRequest* request) {
        if (request->hasParam("cmd")) {
            String command = request->getParam("cmd")->value();
            if (command == "stop") {

                avgSpeedInput = 0;    
                request->send(200, "text/plain", "Robot stopped");
                Serial.println("Robot stopped");
            } else if (command == "accelerate") {
                // Handle accelerate logic increase setpoint v = 0.2 m/s
                avgSpeedInput += 0.2;
                request->send(200, "text/plain", "Robot accelerating");
                Serial.println("Robot accelerating");
            } else if (command == "decelerate") {
                // Handle decelerate logic decrease setpoint v = 0.2 m/s
                avgSpeedInput -= 0.2;
                request->send(200, "text/plain", "Robot decelerating");
                Serial.println("Robot decelerating");
            } else if (command == "left" || command == "right") {
                // Handle turning logic add steer to speed
                steer = (command == "left") ? -10 : 10;
                request->send(200, "text/plain", "Turning " + command);
                Serial.println("Turning " + command);
            } else {
                request->send(400, "text/plain", "Invalid command");
            }
        } else {
            request->send(400, "text/plain", "Missing command parameter");
        }
    });

    // Handle controller type change
    server.on("/controller", HTTP_GET, [](AsyncWebServerRequest* request) {
        // Handle getting the current mode
        if (request->hasParam("get")) {
            String query = request->getParam("get")->value();
            if (query == "current") {
                String currentMode;
                switch (controlMode) {
                    case PID_ANGLE: currentMode = "angle"; break;
                    case PID_POS: currentMode = "pos"; break;
                    case PID_SPEED: currentMode = "speed"; break;
                    default: currentMode = "unknown"; break;
                }
                request->send(200, "text/plain", currentMode);
                return;
            }
        }

        // Handle setting a new mode
        if (request->hasParam("set")) {
            String mode = request->getParam("set")->value();
            if (mode == "angle") {
                controlMode = PID_ANGLE;
                request->send(200, "text/plain", "Mode set to Angle");
                Serial.println("Mode set to Angle");
            } else if (mode == "pos") {
                stepperL->setCurrentPosition(0);
                stepperR->setCurrentPosition(0);
                controlMode = PID_POS;
                request->send(200, "text/plain", "Mode set to Position");
                Serial.println("Mode set to Position");
            } else if (mode == "speed") {
                controlMode = PID_SPEED;
                request->send(200, "text/plain", "Mode set to Speed");
                Serial.println("Mode set to Speed");
            } else {
                request->send(400, "text/plain", "Invalid mode");
                Serial.println("Invalid mode");
            }
        } else {
            request->send(400, "text/plain", "Missing mode parameter");
            Serial.println("Missing mode parameter");
        }
    });

    server.onNotFound([](AsyncWebServerRequest *request) {
        request->send(404, "text/plain", "Not Found");
        request->client()->close(); // Close idle connections
    });
    // Start the server  
    server.begin();  
    Serial.println("Web Server started.");  
}  