#include <Arduino.h>
#include <AccelStepper.h>
#include <Adafruit_NeoPixel.h>

// Define pin assignments
#define stepPin1 18
#define dirPin1 19
#define enablePin1 21
#define BUILTIN_LED 2
#define NEOPIXEL_PIN 16
#define NEOPIXEL_COUNT 1

// Motor configuration
#define MICROSTEPS 4 // Microsteps set on the driver
#define STEPS_PER_REV 200 // Full steps per revolution for the stepper motor
AccelStepper stepper(AccelStepper::DRIVER, stepPin1, dirPin1);

// Neopixel setup
Adafruit_NeoPixel strip(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// Motor speed and acceleration
float rpm = 150.0;      // Desired RPM
float maxSpeed  = (rpm / 60.0) * STEPS_PER_REV * MICROSTEPS; // Steps/sec 
float acceleration = maxSpeed * 10; // Steps/sec^2

void setup() {
  // Initialize serial communication
  Serial.begin(115200);

  // Initialize motor driver
  pinMode(enablePin1, OUTPUT);
  digitalWrite(enablePin1, HIGH); // Disable motor initially

  // Setup the stepper motor
  stepper.setEnablePin(enablePin1);
  stepper.setPinsInverted(false, false, true); // Invert enable pin (active LOW)
  stepper.setAcceleration(acceleration);
  stepper.setMaxSpeed(maxSpeed);

  // Initialize built-in LED
  pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(BUILTIN_LED, LOW);

  // Initialize Neopixel
  strip.begin();
  strip.setBrightness(50); // Adjust brightness (0-255)
  strip.show();            // Turn off all LEDs
}

void setLEDColor(uint8_t r, uint8_t g, uint8_t b) {
  strip.setPixelColor(0, strip.Color(r, g, b));
  strip.show();
}

void loop() {
  // Enable motor and run
  digitalWrite(enablePin1, LOW);
  stepper.moveTo(3200*10); // Move 2000 steps forward
  while (stepper.distanceToGo() != 0) {
    stepper.run();
    int speed = stepper.speed(); // Current speed in steps/sec

    // Adjust LEDs based on speed
    if (speed > 0) {
      digitalWrite(BUILTIN_LED, HIGH);
      setLEDColor(0, 255, 0); // Green for acceleration
    } else if (speed < 0) {
      digitalWrite(BUILTIN_LED, HIGH);
      setLEDColor(255, 0, 0); // Red for deceleration
    } else {
      digitalWrite(BUILTIN_LED, LOW);
      setLEDColor(0, 0, 255); // Blue for stationary
    }
  }

  delay(1000); // Pause for 1 second

  // Move back to the start position
  stepper.moveTo(0);
  while (stepper.distanceToGo() != 0) {
    stepper.run();
    int speed = stepper.speed();

    // Adjust LEDs as before
    if (speed > 0) {
      digitalWrite(BUILTIN_LED, HIGH);
      setLEDColor(0, 255, 0); // Green for acceleration
    } else if (speed < 0) {
      digitalWrite(BUILTIN_LED, HIGH);
      setLEDColor(255, 0, 0); // Red for deceleration
    } else {
      digitalWrite(BUILTIN_LED, LOW);
      setLEDColor(0, 0, 255); // Blue for stationary
    }
  }

  delay(1000); // Pause for 1 second
}
