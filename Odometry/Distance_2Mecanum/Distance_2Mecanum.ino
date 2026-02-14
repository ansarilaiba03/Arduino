#include <Arduino.h>

// ---------------- ENCODER PINS ----------------
const int encoderLeftA = 2;   // Left encoder channel A
const int encoderLeftB = 3;   // Left encoder channel B
const int encoderRightA = 18; // Right encoder channel A
const int encoderRightB = 19; // Right encoder channel B

// ---------------- WHEEL & ENCODER VALUES ----------------
volatile long leftPulses = 0;
volatile long rightPulses = 0;

const int pulsesPerRevLeft = 100;   // your measured value
const int pulsesPerRevRight = 140;  // your measured value

const float wheelDiameter = 0.152;  // 152 mm = 15.2 cm
const float wheelCircumference = PI * wheelDiameter;  // in meters

// ---------------- INTERRUPT FUNCTIONS ----------------
void leftEncoderISR() {
  // Check B channel to determine direction
  if (digitalRead(encoderLeftB) == HIGH)
    leftPulses++;   // Forward
  else
    leftPulses--;   // Backward
}

void rightEncoderISR() {
  // Check B channel to determine direction
  if (digitalRead(encoderRightB) == HIGH)
    rightPulses++;  // Forward
  else
    rightPulses--;  // Backward
}

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(9600);

  pinMode(encoderLeftA, INPUT_PULLUP);
  pinMode(encoderLeftB, INPUT_PULLUP);
  pinMode(encoderRightA, INPUT_PULLUP);
  pinMode(encoderRightB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoderLeftA), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderRightA), rightEncoderISR, RISING);

  Serial.println("Rotate wheels to see distance (forward +, backward -)...");
}

// ---------------- LOOP ----------------
void loop() {
  // Calculate distance from pulses
  float leftRevs = (float)leftPulses / pulsesPerRevLeft;
  float rightRevs = (float)rightPulses / pulsesPerRevRight;

  float leftDistance = leftRevs * wheelCircumference;   // meters
  float rightDistance = rightRevs * wheelCircumference; // meters
  float LinearVelocity = (rightDistance + leftDistance)/2;
  float AngularVelocity = (rightDistance - leftDistance)/2;

  // Print
  
  // Serial.print("Left Distance (m): ");
  // Serial.print(leftDistance, 3);
  // Serial.print(" | Right Distance (m): ");
  // Serial.println(rightDistance, 3);
   Serial.print(" Linear Velocity (m): ");
   Serial.print(LinearVelocity, 3);
   Serial.print(" Angular Velocity (m): ");
   Serial.println(AngularVelocity, 3);


  delay(500); // update every half second
}