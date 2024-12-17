#include <Arduino.h>  
#include <FastAccelStepper.h>   
#include <QuickPID.h>
#include <WiFi.h>  
#include <ESPmDNS.h>
#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>  
#include <AsyncTCP.h>  
#include <WebSocketsServer.h>
#include <TMCStepper.h>
#include <SPIFFS.h>  
#include <ArduinoOTA.h>
#include <Adafruit_NeoPixel.h>
#include "config.h"  
#include "IMUHandler.h"  
#include <fastStepper.h>
#include <PID.h>


/********************************************************************/
// Driving behaviour
float speedFactor = 0.7;  // how strong it reacts to inputs, lower = softer (limits max speed) (between 0 and 1)
float steerFactor = 1.0;  // how strong it reacts to inputs, lower = softer (limits max speed) (between 0 and 1)
void IRAM_ATTR motLeftTimerFunction();
void IRAM_ATTR motRightTimerFunction();
// ste, dir, timer, timerFunction, inverseDir
fastStepper motLeft(MOTORL_STEP_PIN, MOTORL_DIR_PIN, 0, motLeftTimerFunction);
fastStepper motRight(MOTORR_STEP_PIN, MOTORR_DIR_PIN, 1, motRightTimerFunction);

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
void IRAM_ATTR motLeftTimerFunction() {
  portENTER_CRITICAL_ISR(&timerMux);
  motLeft.timerFunction();
  portEXIT_CRITICAL_ISR(&timerMux);
}
void IRAM_ATTR motRightTimerFunction() {
  portENTER_CRITICAL_ISR(&timerMux);
  motRight.timerFunction();
  portEXIT_CRITICAL_ISR(&timerMux);
}



// -- PID control
#define dT_MICROSECONDS 5000
#define dT dT_MICROSECONDS/1000000.0

#define PID_ANGLE_MAX 12
PID pidAngle(cPID, dT, PID_ANGLE_MAX, -PID_ANGLE_MAX);
#define PID_POS_MAX 35
PID pidPos(cPD, dT, PID_POS_MAX, -PID_POS_MAX);
PID pidSpeed(cP, dT, PID_POS_MAX, -PID_POS_MAX);


struct {
  float speed = 0;
  float steer = 0;
  float speedGain = 0.15;
  float steerGain = 0.25;
  float speedOffset = 0.0;
  bool selfRight = 0;
  bool disableControl = 0;
  bool override = 0;
} remoteControl;

float pidAngleOutput = 0 ,pidPosOutput = 0, pidSpeedOutput = 0;
/*****************************************************/
#define DRIVER1_ADDRESS 0b00   // Address for Motor Driver 1
#define DRIVER2_ADDRESS 0b01   // Address for Motor Driver 2

#define SERIAL_PORT Serial1    // Hardware UART for TMC2209 communication
#define R_SENSE 0.11f 
uint8_t microStep = 16;
uint8_t controlMode = PID_POS; // Default to angle+position control  
unsigned long previousMillis = 0;
const unsigned long interval = 1000;

// PID tuning parameters  
float Kp_angle = 8.5, Ki_angle = 0, Kd_angle = 0.05, N_angle = 20; 
float Kp_pos = 20.0, Ki_pos = 0.5, Kd_pos = 0.1, N_pos = 50;  
float Kp_speed = 20.0, Ki_speed = 0.8, Kd_speed = 0.2, N_speed = 20;  
uint8_t angleF = 200, posF = 20, speedF = 20; 


// PID variables  
float input_angle = 0, output_angle = 0, setpoint_angle = 0;  
float input_pos = 0, output_pos = 0, setpoint_pos = 0;  
float input_speed = 0, output_speed = 0, setpoint_speed = 0;  

float avgSpeedInput = 0, avgSteerInput = 0, stepperLSpeed = 0, stepperRSpeed = 0;
float filteredStepperLSpeed = 0, filteredStepperRSpeed = 0;
float alphaSpeed = 0.96;
float tau = 0.5, lastTau = 0.5;
bool stepperDisabled = false;

unsigned long lastWebSocketSend = 0;
const unsigned long webSocketInterval = 100; // Send updates every 100ms (10Hz)

Adafruit_NeoPixel neopixel = Adafruit_NeoPixel(1, NEOPIXEL_PIN);

bool calibrateToggle = true;

union WebSocketData {
    struct {
        float time;
        float angle_setpoint;
        float angle_input;
        float angle_error;
        float angle_output;
        float position_setpoint;
        float position_input;
        float position_output;
        float speed_setpoint;
        float speed_input;
        float speed_output;
        float motor_left_speed;
        float motor_right_speed;
        float robot_position;
    } data;
    uint8_t bytes[sizeof(data)];
};


// PID instances  
// PID pid_angle(&input_angle, &output_angle, &setpoint_angle, Kp_angle, Ki_angle, Kd_angle, 1, REVERSE); 
// PID pid_pos(&input_pos, &output_pos, &setpoint_pos, Kp_pos, Ki_pos, Kd_pos, 1, DIRECT);  
// PID pid_speed(&input_speed, &output_speed, &setpoint_speed, Kp_speed, Ki_speed, Kd_speed, 1, DIRECT);  
QuickPID pid_angle(&input_angle, &output_angle, &setpoint_angle, 
                    Kp_angle, Ki_angle, Kd_angle, 
                    pid_angle.Action::reverse);

QuickPID pid_pos(&input_pos, &output_pos, &setpoint_pos, 
                    Kp_pos, Ki_pos, Kd_pos, 
                    pid_pos.Action::direct);

QuickPID pid_speed(&input_speed, &output_speed, &setpoint_speed,
                    Kp_speed, Ki_speed, Kd_speed,
                    pid_speed.Action::direct);


// TMC2209 Instances
TMC2209Stepper driver1 = TMC2209Stepper(&SERIAL_PORT, R_SENSE, DRIVER1_ADDRESS);
TMC2209Stepper driver2 = TMC2209Stepper(&SERIAL_PORT, R_SENSE, DRIVER2_ADDRESS);
// Stepper instances  
FastAccelStepperEngine engine;  
FastAccelStepper* stepperL = nullptr;  
FastAccelStepper* stepperR = nullptr;  

// IMU instance  
IMUHandler& imu = IMUHandler::getInstance();  

// WiFi and Web Server  
AsyncWebServer server(WEB_SERVER_PORT);  
// AsyncWebSocket ws("/ws");
WebSocketsServer ws = WebSocketsServer(81);

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
    SERIAL_PORT.begin(115200); // UART speed for TMC2209 communication

    // Driver 1 Initialization
    driver1.begin();
    driver1.toff(4);  // Enable SpreadCycle
    driver1.rms_current(1200); // Motor current in mA
    driver1.microsteps(MICROSTEPS);
    driver1.en_spreadCycle(true);

    // Driver 2 Initialization
    driver2.begin();
    driver2.toff(4);  // Enable SpreadCycle
    driver2.rms_current(1200);
    driver2.microsteps(MICROSTEPS);
    driver2.en_spreadCycle(true); 

    // Initialize SPIFFS  
    if (!SPIFFS.begin(true)) {  
        Serial.println("Failed to mount SPIFFS!");  
        while (1);  
    }  

    // Initialize IMU  
    imu.initialize();  
    imu.calibrate(calibrateToggle); 

    pinMode(MOTORL_ENABLE_PIN, OUTPUT);
    pinMode(MOTORR_ENABLE_PIN, OUTPUT);


    digitalWrite(MOTORL_ENABLE_PIN, HIGH);
    digitalWrite(MOTORR_ENABLE_PIN, HIGH); //disable for now

    motLeft.init();
    motRight.init();

    motLeft.microStep = microStep;
    motRight.microStep = microStep; 

    Kp_angle = 0.65;
    Ki_angle = 1.0;
    Kd_angle = 0.075;
    N_angle = 15;

    Kp_pos = 1;
    Ki_pos = 0;
    Kd_pos = 1.2;
    N_pos = 50;

    Kp_speed = 6;
    Ki_speed = 5;
    Kd_speed = 0;
    N_speed = 20;

    
    pidAngle.setParameters(Kp_angle, Ki_angle, Kd_angle, N_angle);
    pidPos.setParameters(Kp_pos, Ki_pos, Kd_pos, N_pos);
    pidSpeed.setParameters(Kp_speed, Ki_speed, Kd_speed, N_speed);

    // // Initialize stepper engine  
    // engine.init();

    // // Initialize motors  
    // stepperL = engine.stepperConnectToPin(MOTORL_STEP_PIN);  
    // if (!stepperL) {  
    //     Serial.println("Motor connection failed!");  
    //     while (1);  
    // }  
    // stepperL->setDirectionPin(MOTORL_DIR_PIN);  
    // stepperL->setEnablePin(MOTORL_ENABLE_PIN);  
    // stepperL->enableOutputs();  
    // stepperL->setSpeedInHz(1);  
    // stepperL-> setCurrentPosition(0);
    // stepperL->setAcceleration(MAX_SPEED/tau);  
    // Serial.println("Motor Initialized");

    // stepperR = engine.stepperConnectToPin(MOTORR_STEP_PIN);
    // if (!stepperR) {
    //     Serial.println("Motor connection failed!");
    //     while (1);
    // }
    // stepperR->setDirectionPin(MOTORR_DIR_PIN);
    // stepperR->setEnablePin(MOTORR_ENABLE_PIN);
    // stepperR->enableOutputs();
    // stepperR->setSpeedInHz(1);
    // stepperR->setCurrentPosition(0);
    // stepperR->setAcceleration(MAX_SPEED/tau);
    // Serial.println("Motor Initialized");



    // // Initialize PID controllers  
    // pid_angle.SetMode(pid_angle.Control::automatic);  
    // pid_pos.SetMode(pid_pos.Control::automatic);  
    // pid_speed.SetMode(pid_speed.Control::automatic);
  

    // pid_angle.SetOutputLimits(-MAX_SPEED_RPM, MAX_SPEED_RPM);  
    // pid_pos.SetOutputLimits(-MAX_TILT_ANGLE, MAX_TILT_ANGLE);  
    // pid_speed.SetOutputLimits(-MAX_TILT_ANGLE, MAX_TILT_ANGLE);  

    // pid_angle.SetSampleTimeUs(1000/angleF *  1000);  // 200 Hz 
    // pid_pos.SetSampleTimeUs(1000/posF * 1000);  // 50 Hz
    // pid_speed.SetSampleTimeUs(1000/speedF * 1000);  // 50 Hz


    // set neopixel pin green
    neopixel.begin();
    neopixel.setPixelColor(0, neopixel.Color(0, 255, 0));
    neopixel.show();

    

    // Setup WiFi and Web Server  
    setupWiFi();  
    setupWebServer();  
    setupOTA();

    Serial.println("System Initialized. Use Serial or Web Interface to adjust parameters.");  
}  

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
 return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void loop() {  

        // Process Serial commands  
    processSerialCommands();  
 

    // Update IMU data  
    imu.update();  
    input_angle = imu.getRoll() * RAD_TO_DEG;
    /**************************************************** */
    static unsigned long tLast = 0;
    pidAngleOutput = 0;
    float avgMotSpeed;
    float steer = 0;
    static float avgSteer;
    static float avgSpeed;
    static boolean enableControl = 0;
    static float avgMotSpeedSum = 0;
    int32_t avgMotStep;
    pidPosOutput = 0, pidSpeedOutput = 0;
    static uint8_t k = 0;
    static uint32_t lastInputTime = 0;
    uint32_t tNowMs;
    float absSpeed = 0;
    float noiseValue = 0;
    static boolean overrideMode = 0, lastOverrideMode = 0;
    static boolean selfRight = 0;
    static boolean disableControl = 0;
    static float angleErrorIntegral = 0;
    unsigned long tNow = micros();
    tNowMs = millis();
    remoteControl.speed = avgSpeedInput;
    remoteControl.steer = avgSteerInput;

    avgSpeed = alphaSpeed * avgSpeed + (1-alphaSpeed)*remoteControl.speed;
    avgSteer = alphaSpeed * avgSteer + (1-alphaSpeed)*remoteControl.steer;

    if (remoteControl.selfRight && !enableControl) { // Start self-right action (stops when robot is upright)
      selfRight = 1;
      disableControl = 0;
      remoteControl.selfRight = 0; // Reset single action bool
    } else if (remoteControl.disableControl && enableControl ) { // Sort of kill-switch
      disableControl = 1;
      selfRight = 0;
      remoteControl.disableControl = 0;
    }

    if (enableControl) {

      if (abs(avgSpeed)<0.2) {
        // remoteControl.speed = 0;
      } else {
        lastInputTime = tNowMs;
        if (controlMode==PID_POS) {
          controlMode = PID_SPEED;
          motLeft.setStep(0);
          motRight.setStep(0);
          pidSpeed.reset();
        }
      }

      steer = avgSteer;

    // Switch to position control if no input is received for a certain amount of time
      if (tNowMs-lastInputTime>2000 && controlMode == PID_SPEED) {
        controlMode = 1;
        motLeft.setStep(0);
        motRight.setStep(0);
        pidPos.reset();
      }

      // Actual controller computations
      if (controlMode == 0) {
        pidAngle.setpoint = avgSpeed*2;
      } else if (controlMode == 1) {
        avgMotStep = (motLeft.getStep() + motRight.getStep())/2;
        pidPos.setpoint = avgSpeed;
        pidPos.input = -((float) avgMotStep) / 1000.0;
        pidPosOutput = pidPos.calculate();
        pidAngle.setpoint = pidPosOutput;
      } else if (controlMode == 2) {
        pidSpeed.setpoint = avgSpeed;
        pidSpeed.input = -avgMotSpeedSum/100.0;
        pidSpeedOutput = pidSpeed.calculate();
        pidAngle.setpoint = pidSpeedOutput;
      }


      pidAngle.input = input_angle;

      pidAngleOutput = pidAngle.calculate();


      avgMotSpeedSum += pidAngleOutput/2;
      avgMotSpeedSum = constrain(avgMotSpeedSum, -1500, 1500);
      avgMotSpeed = avgMotSpeedSum;
      motLeft.speed = avgMotSpeed + steer;
      motRight.speed = avgMotSpeed - steer;

      // Detect if robot has fallen. Concept: integrate angle controller error over time. 
      // If absolute integrated error surpasses threshold, disable controller
      angleErrorIntegral += (pidAngle.setpoint - pidAngle.input) * dT;
      if (selfRight) {
        if (abs(angleErrorIntegral) > 30*0.3) {
          selfRight = 0;
          disableControl = 1;
        }
      } else {
        if (abs(angleErrorIntegral) > 30) {
          disableControl = 1;
        }
      }


      // Switch microstepping
      absSpeed = abs(avgMotSpeed);
      uint8_t lastMicroStep = microStep;

      if (absSpeed > (150 * 32 / microStep) && microStep > 1) microStep /= 2;
      if (absSpeed < (130 * 32 / microStep) && microStep < 32) microStep *= 2;

      // if (microStep!=lastMicroStep) {
      //   motLeft.microStep = microStep;
      //   motRight.microStep = microStep;
      //   setMicroStep(microStep);
      // }

      // Disable control if robot is almost horizontal. Re-enable if upright.
      if ((abs(input_angle)>EMERGENCY_STOP_ANGLE && !selfRight) || disableControl) {
        enableControl = 0;
        overrideMode = 0;
        // disableControl = 0; // Reset disableControl flag
        motLeft.speed = 0;
        motRight.speed = 0;
        digitalWrite(MOTORL_ENABLE_PIN, 1); // Inverted action on enable pin
        digitalWrite(MOTORR_ENABLE_PIN, 1); // Inverted action on enable pin

        neopixel.setPixelColor(0, neopixel.Color(255, 0, 0));
        neopixel.show();
      }
      if (abs(input_angle)<EMERGENCY_STOP_ANGLE && selfRight) {
        selfRight = 0;
        angleErrorIntegral = 0; // Reset, otherwise the fall detection will be triggered immediately
      }
    } else { // Control not active

      // Override control
      if (remoteControl.override) { // Transition from disable to enable
        remoteControl.override = 0;
        // Enable override mode
        motLeft.speed = 0;
        motRight.speed = 0;
        digitalWrite(MOTORL_ENABLE_PIN, 0); // Inverted action on enable pin
        digitalWrite(MOTORR_ENABLE_PIN, 0); // Inverted action on enable pin
        overrideMode = 1;
      }

      if (remoteControl.disableControl) {
        remoteControl.disableControl = 0;
        
        digitalWrite(MOTORL_ENABLE_PIN, 1); // Inverted action on enable pin
        digitalWrite(MOTORR_ENABLE_PIN, 1); // Inverted action on enable pin

        overrideMode = 0;
      }

      if (abs(input_angle)>5+5) { // Only reset disableControl flag if angle is out of "enable" zone, otherwise robot will keep cycling between enable and disable states
        disableControl = 0; 
      }

      if ((abs(input_angle)<5 || selfRight) && !disableControl) { // (re-)enable and reset stuff
        enableControl = 1;
        neopixel.setPixelColor(0, neopixel.Color(0, 255, 0));
        neopixel.show();

        controlMode = 1;
        // avgMotSpeedSum = 0;

        if (!overrideMode) {
          avgMotSpeedSum = 0;
          digitalWrite(MOTORL_ENABLE_PIN, 0); // Inverted action on enable pin
          digitalWrite(MOTORR_ENABLE_PIN, 0); // Inverted action on enable pin
          pidAngle.reset();
        } else {
          avgMotSpeedSum = (motLeft.speed + motRight.speed) / 2;
          overrideMode = 0;
        }

        motLeft.setStep(0);
        motRight.setStep(0);
        pidPos.reset();
        pidSpeed.reset();
        
        angleErrorIntegral = 0;
        // delay(1);
      }

      if (overrideMode) {
        float spd = avgSpeed;
        float str = avgSteer;
        // if (spd<3) spd = 0;
        // if (str<3) str = 0;
        motLeft.speed = -30*spd + 2*str;
        motRight.speed = -30*spd - 2*str;

        // Run angle PID controller in background, such that it matches when controller takes over, if needed
        pidAngle.input = input_angle;
        pidAngleOutput = pidAngle.calculate();
        // pidSpeed.setpoint = avgSpeed;
        // pidSpeed.input = -(motLeft.speed+motRight.speed)/2/100.0;
        // pidSpeedOutput = pidSpeed.calculate();
      }
      // Serial << motLeft.speed << "\t" << motRight.speed << "\t" << overrideMode << endl;
    }

    motLeft.update();
    motRight.update();
    /********************************************************** */
    
    // Process Serial commands  
    processSerialCommands();  
    sendWebSocketData();
    ws.loop(); // Handle WebSocket events
 

    // // Update IMU data  
    // imu.update();  
    // input_angle = imu.getRoll() * RAD_TO_DEG;
    // input_speed = getRobotSpeed();


    // // Update control mode  
    // updateControlMode(); 
    // stepperLSpeed = output_angle + steer;  // speed in rpm
    // stepperRSpeed = output_angle - steer;  // speed in rpm

    // // low-pass fileter for speed
    // filteredStepperLSpeed = (1 - alphaSpeed) * filteredStepperLSpeed + (alphaSpeed) * stepperLSpeed;
    // filteredStepperRSpeed = (1 - alphaSpeed) * filteredStepperLSpeed + (alphaSpeed) * stepperLSpeed;

    // if (near(steer, 0, 1)) {
    //     filteredStepperLSpeed = filteredStepperRSpeed;
    // }

    // stepperL->setSpeedInHz(abs(rpm2sps(filteredStepperLSpeed)));
    // stepperR->setSpeedInHz(abs(rpm2sps(filteredStepperRSpeed)));

    // if (lastTau != tau) {
    //     stepperL->setAcceleration(MAX_SPEED/tau);
    //     stepperR->setAcceleration(MAX_SPEED/tau);
    //     lastTau = tau;
    // }
    
    // if (stepperLSpeed == 0) stepperL->stopMove();
    // else if (stepperLSpeed > 0) stepperL->runForward();
    // else stepperL->runBackward();

    // if (stepperRSpeed == 0) stepperR->stopMove();
    // else if (stepperRSpeed > 0) stepperR->runBackward();
    // else stepperR->runForward();
    

    // sendWebSocketData();
    // // pid_angle.Compute();
    // // // ArduinoOTA.handle(); // Handle OTA updates
    // ws.loop(); // Handle WebSocket events



    // unsigned long currentMillis = millis();
    // if (currentMillis - previousMillis >= interval) {
    //     previousMillis = currentMillis;

    //     // Emergency stop if tilt angle exceeds safety limits  
    //     if (abs(input_angle) > EMERGENCY_STOP_ANGLE) {  
    //         if (!stepperDisabled) {
    //             stepperL->setDelayToDisable(50);
    //             stepperR->setDelayToDisable(50);
    //             stepperL->disableOutputs();
    //             stepperR->disableOutputs();
    //             stepperDisabled = true;

    //             // neopixel red
    //             neopixel.setPixelColor(0, neopixel.Color(255, 0, 0));
    //             neopixel.show();
    //             Serial.println("Emergency Stop: Tilt angle exceeded safety limits!");  
    //     }}

    //     if (stepperDisabled && abs(input_angle) < EMERGENCY_STOP_ANGLE) {  
    //         stepperL->setDelayToEnable(500);
    //         stepperR->setDelayToEnable(500);

    //         stepperL->enableOutputs();
    //         stepperR->enableOutputs();

    //         stepperDisabled = false;

    //         // neopixel green
    //         neopixel.setPixelColor(0, neopixel.Color(0, 255, 0));
    //         neopixel.show();
    //         Serial.println("Emergency Stop: Resumed operation");  
    //     }

    //     if (WiFi.status() != WL_CONNECTED) {
    //         Serial.print("Reconnecting to WiFi...");
    //         setupWiFi();
    //         delay(1000);
    //     }
    // }
}  

void updateControlMode() {
    switch (controlMode) {
        case PID_ANGLE:
            // Angle-only control
            setpoint_angle = avgSpeedInput;  // Speed controller sets angle target
            break;

        case PID_POS:
            // Position control
            // pid_speed.Compute();
            setpoint_pos = avgSpeedInput; //output_speed;  // Speed controller sets position target
            input_pos = getRobotPos();
            

            pid_pos.Compute();
            setpoint_angle = output_pos ; // Position controller sets angle target, output position in m and angle in deg
            break;

        case PID_SPEED:
            // Speed control
            setpoint_speed = avgSpeedInput;  // Speed controller sets speed target
            pid_speed.Compute();
            setpoint_angle = output_speed;  // Speed controller sets angle target, output speed in m/s and angle in deg
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
    server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {  
        request->send(SPIFFS, "/index.html", "text/html");  
    });  

    server.on("/plot", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(SPIFFS, "/plotUnion.html", "text/html");
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
                // pid_angle.SetTunings(Kp, Ki, Kd);
                pidAngle.K = Kp;
                pidAngle.Ti = Ki;
                pidAngle.Td = Kd;

                pidAngle.updateParameters();
                Serial.printf("Updated Angle PID: Kp=%.2f, Ki=%.2f, Kd=%.2f\n", Kp, Ki, Kd);
            } else if (pidType == "pos") {
                // pid_pos.SetTunings(Kp, Ki, Kd);
                pidPos.K = Kp;
                pidPos.Ti = Ki;
                pidPos.Td = Kd;

                pidPos.updateParameters();
                Serial.printf("Updated Position PID: Kp=%.2f, Ki=%.2f, Kd=%.2f\n", Kp, Ki, Kd);
            } else if (pidType == "speed") {
                // pid_speed.SetTunings(Kp, Ki, Kd);
                pidSpeed.K = Kp;
                pidSpeed.Ti = Ki;
                pidSpeed.Td = Kd;

                pidSpeed.updateParameters();
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

                avgSteerInput = 0;    
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
                avgSteerInput += (command == "left") ? -10 : 10;
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
                // pid_angle.Reset();
                pidAngle.reset();

                // neopixel blue
                neopixel.setPixelColor(0, neopixel.Color(0, 0, 100));
                neopixel.show();

                request->send(200, "text/plain", "Mode set to Angle");
                Serial.println("Mode set to Angle");
            } else if (mode == "pos") {
                motLeft.setStep(0);
                motRight.setStep(0);
                controlMode = PID_POS;
                pidPos.reset();
                // magenta
                neopixel.setPixelColor(0, neopixel.Color(100, 0, 100));
                neopixel.show();


                request->send(200, "text/plain", "Mode set to Position");
                Serial.println("Mode set to Position");
            } else if (mode == "speed") {
                controlMode = PID_SPEED;
                pidSpeed.reset();

                //yellow
                neopixel.setPixelColor(0, neopixel.Color(100, 100, 0));
                neopixel.show();

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

    server.on("/setParam", HTTP_GET, [](AsyncWebServerRequest* request) {
        if (request->hasParam("alphaSpeed")) {
            alphaSpeed = request->getParam("alphaSpeed")->value().toFloat();
            // reset the robot position
            motLeft.setStep(0);
            motRight.setStep(0);
            request->send(200, "text/plain", "alphaSpeed set to " + String(alphaSpeed));
            Serial.println("Pos reset, alphaSpeed set to " + String(alphaSpeed));

        }
        if (request->hasParam("tau")){
            tau = request->getParam("tau")->value().toFloat();
            request->send(200, "text/plain", "tau set to " + String(tau));
            Serial.println("tau set to " + String(tau));
        } 
        if (request->hasParam("angleF") || request->hasParam("posF") || request->hasParam("speedF")) {
            angleF = request->getParam("angleF")->value().toFloat();
            posF = request->getParam("posF")->value().toFloat();
            speedF = request->getParam("speedF")->value().toFloat();
            if (angleF != 200 || posF != 20 || speedF != 20) {
                // pid_angle.SetSampleTimeUs(1000 * 1000 / angleF);
                // pid_pos.SetSampleTimeUs(1000 * 1000 / posF);
                // pid_speed.SetSampleTimeUs(1000 * 1000 / speedF);
                request->send(200, "text/plain", "Sample time set to angleF: " + String(angleF) + " posF: " + String(posF) + " speedF: " + String(speedF));
                Serial.println("Sample time set to angleF: " + String(angleF) + " posF: " + String(posF) + " speedF: " + String(speedF));
            }

        }
        else {
            request->send(400, "text/plain", "Missing alphaSpeed parameter");
            Serial.println("Missing alphaSpeed parameter");
        }
    });

    server.on("/calibrate", HTTP_GET, [](AsyncWebServerRequest* request) {
        if (request->hasParam("toggle")) {
            // calibrateToggle = true;
            // imu.initialize();
            // imu.calibrate(calibrateToggle);
            request->send(200, "text/plain", "Calibration toggled");
            Serial.println("Calibration Done");
        } else {
            request->send(400, "text/plain", "Missing toggle parameter");
        }
    });

    server.on("/resetPID", HTTP_GET, [](AsyncWebServerRequest* request) {
        
        // pid_angle.Reset();
        // pid_pos.Reset();
        // pid_speed.Reset();
        pidAngle.reset();
        pidPos.reset();
        pidSpeed.reset();

        request->send(200, "text/plain", "PID controllers reset");
        Serial.println("PID controllers reset");
    });

    

    // ws.begin();

    // ws.onEvent([](AsyncWebSocket* server, AsyncWebSocketClient* client, AwsEventType type,
    //               void* arg, uint8_t* data, size_t len) {
    //     if (type == WS_EVT_CONNECT) {
    //         Serial.printf("Client %u connected\n", client->id());
    //     } else if (type == WS_EVT_DISCONNECT) {
    //         Serial.printf("Client %u disconnected\n", client->id());
    //     }
    // });
    

    server.onNotFound([](AsyncWebServerRequest *request) {
        request->send(404, "text/plain", "Not Found");
        request->client()->close(); // Close idle connections
    });

    ws.begin();
    ws.onEvent([](uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
        if (type == WStype_TEXT) {
            // Serial.println("Text data received.");
        } else if (type == WStype_BIN) {
            Serial.println("Binary data received.");
        }
    });

    // server.addHandler(&ws);
    server.begin();  
    Serial.println("Web Server started.");  
}  

// void sendWebSocketData() {
//     unsigned long currentMillis = millis();
//     if (currentMillis - lastWebSocketSend >= webSocketInterval) {
//         lastWebSocketSend = currentMillis;

//         if (ws.count() > 0) {
//             JsonDocument json;

//             json["time"] = millis() / 1000.0;
//             json["angle"]["setpoint"] = setpoint_angle;
//             json["angle"]["input"] = input_angle;
//             json["angle"]["output"] = output_angle;

//             json["position"]["setpoint"] = setpoint_pos;
//             json["position"]["input"] = input_pos;
//             json["position"]["output"] = output_pos;

//             json["speed"]["setpoint"] = setpoint_speed;
//             json["speed"]["input"] = input_speed;
//             json["speed"]["output"] = output_speed;

//             json["motor"]["left_speed"] = stepperLSpeed;
//             json["motor"]["right_speed"] = stepperRSpeed;
//             json["robot_position"] = getRobotPos();

//             String message;
//             serializeJson(json, message);
//             ws.textAll(message);
//         }
//     }
// }

void sendWebSocketData() {
    unsigned long currentMillis = millis();
    if (currentMillis - lastWebSocketSend >= webSocketInterval) {
        lastWebSocketSend = currentMillis;
        static WebSocketData wsData;
        wsData.data.time = millis() / 1000.0f;
        wsData.data.angle_setpoint = pidAngle.setpoint; //pid * deg; max = tilt angle
        wsData.data.angle_input = input_angle;  //deg; max = 100 deg for plot
        wsData.data.angle_error = setpoint_angle - input_angle;
        wsData.data.angle_output = pidAngleOutput; //pid * deg
        wsData.data.position_setpoint = pidPos.setpoint; //cm 
        wsData.data.position_input = pidPos.input;
        wsData.data.position_output = pidPosOutput;
        wsData.data.speed_setpoint = pidSpeed.setpoint; //m/s
        wsData.data.speed_input = pidSpeed.input;
        wsData.data.speed_output = pidSpeedOutput;
        wsData.data.motor_left_speed = motLeft.speed / 15; //rpm
        wsData.data.motor_right_speed = motRight.speed / 15; //rpm
        wsData.data.robot_position = pidPos.input; //cm

        if (ws.connectedClients(0) > 0) {
            ws.sendBIN(0, wsData.bytes, sizeof(wsData.bytes));
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
