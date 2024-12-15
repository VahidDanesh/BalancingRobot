#include <Arduino.h>  
#include <FastAccelStepper.h>  
#include <PID_v1.h>  
#include <WiFi.h>  
#include <ESPmDNS.h>
#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>  
#include <AsyncTCP.h>  
#include <SPIFFS.h>  
#include <ArduinoOTA.h>
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
float filteredStepperLSpeed = 0, filteredStepperRSpeed = 0;
float alphaSpeed = 0.9;

unsigned long lastWebSocketSend = 0;
const unsigned long webSocketInterval = 100; // Send updates every 100ms (10Hz)


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
AsyncWebSocket ws("/ws");

// Function declarations  
void setupWiFi();  
void setupWebServer();  
void sendWebSocketData();
void processSerialCommands();  
float rpm2sps(float rpm);
float getRobotPos();
float getRobotSpeed();
bool near(float a, float b, float tolerance);
void setRobotSpeed(float stepperLSpeed, float stepperRSpeed);
void updateControlMode();  
void setupOTA();

void setup() {  
    Serial.begin(SERIAL_BAUD);  

    // Initialize SPIFFS  
    // if (!SPIFFS.begin(true)) {  
    //     Serial.println("Failed to mount SPIFFS!");  
    //     while (1);  
    // }  

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
    pid_pos.SetSampleTime(100);  // 10 Hz
    pid_speed.SetSampleTime(100);  // 10 Hz

    // Setup WiFi and Web Server  
    setupWiFi();  
    setupWebServer();  
    setupOTA();

    Serial.println("System Initialized. Use Serial or Web Interface to adjust parameters.");  
}  

void loop() {  
    // Process Serial commands  
    processSerialCommands();  
 

    // Update IMU data  
    imu.update();  
    input_angle = imu.getRoll() * RAD_TO_DEG;
    input_speed = getRobotSpeed();


    // Update control mode  
    updateControlMode(); 
    stepperLSpeed = output_angle + steer;  // speed in rpm
    stepperRSpeed = output_angle - steer;  // speed in rpm

    // low-pass fileter for speed
    filteredStepperLSpeed = (1 - alphaSpeed) * filteredStepperLSpeed + (alphaSpeed) * stepperLSpeed;
    filteredStepperRSpeed = (1 - alphaSpeed) * filteredStepperLSpeed + (alphaSpeed) * stepperLSpeed;

    if (near(steer, 0, 1)) {
        filteredStepperLSpeed = filteredStepperRSpeed;
    }
    stepperL->setSpeedInHz(abs(rpm2sps(filteredStepperLSpeed)));
    stepperR->setSpeedInHz(abs(rpm2sps(filteredStepperRSpeed)));

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
    

    sendWebSocketData();
    ArduinoOTA.handle(); // Handle OTA updates



    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;

        // Emergency stop if tilt angle exceeds safety limits  
        if (abs(input_angle) > EMERGENCY_STOP_ANGLE) {  
            stepperL->setSpeedInHz(1);
            stepperL->runForward();
            stepperR->setSpeedInHz(1);
            stepperR->runBackward();
            Serial.println("Emergency Stop: Tilt angle exceeded safety limits!");  
        }  
        if (WiFi.status() != WL_CONNECTED) {
            Serial.print("Reconnecting to WiFi...");
            Serial.println(WIFI_SSID);
            WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
        }
    }
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
            input_pos = getRobotPos();
            

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

float getRobotPos() {
    float motPos = (stepperL->getCurrentPosition() - stepperR->getCurrentPosition()) / 2.0;
    return motPos / (STEPS_PER_REV * MICROSTEPS) * TWO_PI * WHEEL_RADIUS;
}

float getRobotSpeed() {
    return (stepperL->getCurrentSpeedInMilliHz() - stepperR->getCurrentSpeedInMilliHz()) /
           (1000.0 * STEPS_PER_REV * MICROSTEPS) * PI * WHEEL_RADIUS;   // ((w1-w2)/2) * r
}

bool near(float value, float target, float tolerance) {
    return abs(value - target) < tolerance;
}

// void setRobotSpeed(float stepperLSpeed, float stepperRSpeed) {  
//     stepperLSpeed = (rpm2sps(stepperLSpeed));
//     stepperRSpeed = (rpm2sps(stepperRSpeed));

//     if (!stepperL->setSpeedInHz(abs(stepperLSpeed))) {
//         stepperL->stopMove();
//         Serial.println("Error setting speed for left motor!");
//         Serial.println(stepperLSpeed);
//     } else if (!stepperR->setSpeedInHz(abs(stepperRSpeed))) {
//         stepperR->stopMove();
//         Serial.println("Error setting speed for right motor!");
//         Serial.println(stepperRSpeed);
//     }

//     if (stepperLSpeed > 0){
//         stepperL->runForward();
//     } else {
//         stepperL->runBackward();
//     }
//     if (stepperRSpeed > 0){
//         stepperR->runBackward();
//     } else {
//         stepperR->runForward();
//     }

// }



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
        } else if (command.startsWith("a")) {
            alphaSpeed = command.substring(1).toFloat();
            Serial.print("Updated alphaSpeed: ");
            Serial.println(alphaSpeed);
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
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Headers", "Content-Type");


    // Serve the index.html file  
    // server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {  
    //     request->send(SPIFFS, "/index.html", "text/html");  
    // });  

    // server.on("/plot", HTTP_GET, [](AsyncWebServerRequest* request) {
    //     request->send(SPIFFS, "/plot.html", "text/html");
    // });

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
                steer = 0;    
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

    ws.onEvent([](AsyncWebSocket* server, AsyncWebSocketClient* client, AwsEventType type,
                  void* arg, uint8_t* data, size_t len) {
        if (type == WS_EVT_CONNECT) {
            Serial.printf("Client %u connected\n", client->id());
        } else if (type == WS_EVT_DISCONNECT) {
            Serial.printf("Client %u disconnected\n", client->id());
        }
    });
    

    server.onNotFound([](AsyncWebServerRequest *request) {
        request->send(404, "text/plain", "Not Found");
        request->client()->close(); // Close idle connections
    });
    // Start the server  
    server.addHandler(&ws);
    server.begin();  
    Serial.println("Web Server started.");  
}  

void sendWebSocketData() {
    unsigned long currentMillis = millis();
    if (currentMillis - lastWebSocketSend >= webSocketInterval) {
        lastWebSocketSend = currentMillis;

        if (ws.count() > 0) {
            JsonDocument json;

            json["time"] = millis() / 1000.0;
            json["angle"]["setpoint"] = setpoint_angle;
            json["angle"]["input"] = input_angle;
            json["angle"]["output"] = output_angle;

            json["position"]["setpoint"] = setpoint_pos;
            json["position"]["input"] = input_pos;
            json["position"]["output"] = output_pos;

            json["speed"]["setpoint"] = setpoint_speed;
            json["speed"]["input"] = input_speed;
            json["speed"]["output"] = output_speed;

            json["motor"]["left_speed"] = stepperLSpeed;
            json["motor"]["right_speed"] = stepperRSpeed;
            json["robot_position"] = getRobotPos();

            String message;
            serializeJson(json, message);
            ws.textAll(message);
        }
    }
}

void setupOTA() {
    ArduinoOTA.setHostname("robot"); // Optional, set a custom hostname
    ArduinoOTA.setPassword(WIFI_PASSWORD); // Optional, set a password for OTA updates

    // What to do before OTA starts
    ArduinoOTA.onStart([]() {
        String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
        Serial.println("OTA Start: " + type);
    });

    // What to do when OTA ends
    ArduinoOTA.onEnd([]() {
        Serial.println("\nOTA End");
    });

    // OTA progress
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress * 100) / total);
    });

    // OTA error handling
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

    ArduinoOTA.begin();
    Serial.println("OTA Ready. IP Address: " + WiFi.localIP().toString());
}
