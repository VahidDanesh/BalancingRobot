#include <Arduino.h>
#include <FlySkyIBus.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <Streaming.h>
#include <MPU6050.h>
#include <PID.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WebSocketsServer.h>
#include <FS.h>
#include <SPIFFS.h>
#include <SPIFFSEditor.h>
#include <fastStepper.h>
#include <Preferences.h>  // for storing settings
#include "config.h"       // Include the configuration file

// ----- Function prototypes
void connectToWiFi();
void startAPMode();
void printWebServerInstructions();
void handleSPIFFS();
void handleWiFiError();
void sendWifiList(void);
void parseSerial();
void parseCommand(char* data, uint8_t length);
void calculateGyroOffset(uint8_t nSample);
void readSensor();
void initSensor(uint8_t n);
void setMicroStep(uint8_t uStep);
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length);
void sendConfigurationData(uint8_t num);

// ----- Definitions and variables
AsyncWebServer httpServer(80);
WebSocketsServer wsServer = WebSocketsServer(81);
Preferences preferences;

// Stepper motors
fastStepper motLeft(18, 19, 0, motLeftTimerFunction); // Step, Dir, Timer, TimerFunction
fastStepper motRight(2, 15, 1, motRightTimerFunction); // Step, Dir, Timer, TimerFunction

// IMU
MPU6050 imu(MPU6050_ADDRESS_AD0_HIGH); // AD0_HIGH = 0x69, AD0_LOW = 0x68

// ----- Main code
void setup() {
  Serial.begin(115200);
  Serial.println("\nStarting Balancing Robot...");

  // Initialize SPIFFS
  handleSPIFFS();

  // Initialize WiFi
  connectToWiFi();

  // Initialize motors
  pinMode(MOT_ENABLE_PIN, OUTPUT);
  digitalWrite(MOT_ENABLE_PIN, disableMot ^ activeLOW); // Disable motors during startup
  motLeft.init();
  motRight.init();

  // Initialize IMU
  Serial.println("Initializing MPU6050...");
  Wire.begin(21, 22, 400000UL);
  imu.initialize();
  if (imu.testConnection()) {
    Serial.println("MPU6050 connection successful!");
  } else {
    Serial.println("MPU6050 connection failed! Check wiring.");
  }

  // Initialize Web Server
  httpServer.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    Serial.println("Serving index.htm...");
    request->send(SPIFFS, "/index.htm");
  });

  httpServer.serveStatic("/", SPIFFS, "/");
  httpServer.onNotFound([](AsyncWebServerRequest *request) {
    request->send(404, "text/plain", "File Not Found");
  });

  httpServer.addHandler(new SPIFFSEditor(SPIFFS, HTTP_USERNAME, HTTP_PASSWORD));
  httpServer.begin();

  wsServer.begin();
  wsServer.onEvent(webSocketEvent);

  Serial.println("Web server started.");
  printWebServerInstructions();

  // Initialize OTA
  ArduinoOTA.setHostname(HOSTNAME);
  ArduinoOTA.begin();
  Serial.println("OTA updates enabled.");

  Serial.println("Setup complete. Robot is ready.");
}

void loop() {
  // Handle OTA updates
  ArduinoOTA.handle();

  // WebSocket server loop
  wsServer.loop();

  // Other tasks...
}

// ----- WiFi Connection -----
void connectToWiFi() {
  Serial.println("Connecting to WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long startAttemptTime = millis();
  const unsigned long timeout = 10000; // 10 seconds timeout

  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < timeout) {
    Serial.print(".");
    delay(500);
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWiFi connection failed. Starting AP mode...");
    startAPMode();
  }
}

void startAPMode() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);

  Serial.println("AP mode started.");
  Serial.print("AP IP Address: ");
  Serial.println(WiFi.softAPIP());
}

// ----- SPIFFS Handling -----
void handleSPIFFS() {
  if (!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)) {
    Serial.println("SPIFFS mount failed! Check file system.");
    while (true); // Halt execution
  } else {
    Serial.println("SPIFFS mounted successfully.");
  }
}

// ----- Web Server Instructions -----
void printWebServerInstructions() {
  Serial.println("\n--- Web Server Instructions ---");
  Serial.println("1. Open a web browser.");
  Serial.println("2. Enter the following URL:");
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("   http://");
    Serial.println(WiFi.localIP());
  } else {
    Serial.print("   http://");
    Serial.println(WiFi.softAPIP());
  }
  Serial.println("3. Access the index.htm file to configure the robot.");
  Serial.println("4. Use the web interface to monitor and control the robot.");
  Serial.println("--------------------------------");
}

// ----- Error Handling -----
void handleWiFiError() {
  Serial.println("Error: Unable to connect to WiFi.");
  Serial.println("Check your WiFi credentials in config.h.");
  Serial.println("Restarting in AP mode...");
  startAPMode();
}

// Other functions (PID, IMU, motor control, etc.) remain unchanged.