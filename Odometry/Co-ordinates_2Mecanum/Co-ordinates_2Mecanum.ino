#include <Arduino.h>

// ---------------- ENCODER PINS ----------------
const int encoderLeftA = 2;     // Left encoder channel A
const int encoderLeftB = 3;     // Left encoder channel B
const int encoderRightA = 18;   // Right encoder channel A
const int encoderRightB = 19;   // Right encoder channel B

// ---------------- ROBOT CONSTANTS ----------------
const float pulsesPerRevLeft = 100.0;    // measured
const float pulsesPerRevRight = 140.0;   // measured
const float wheelDiameter = 152.0;       // 
const float wheelBase = 325.0;           // 
const float wheelCircumference = PI * wheelDiameter;  // mm

// ---------------- ENCODER COUNTS ----------------
volatile long leftPulses = 0;
volatile long rightPulses = 0;

// ---------------- POSITION VARIABLES ----------------
float x = 0.0;     // meters
float y = 0.0;     // meters
float theta = 0.0; // degrees (orientation)

long prevLeftPulses = 0;
long prevRightPulses = 0;

// ---------------- INTERRUPT SERVICE ROUTINES ----------------
void leftEncoderISR() {
  if (digitalRead(encoderLeftB) == HIGH)
    leftPulses++;
  else
    leftPulses--;
}

void rightEncoderISR() {
  if (digitalRead(encoderRightB) == HIGH)
    rightPulses++;
  else
    rightPulses--;
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

  Serial.println("Odometry Started...  ");
  Serial.println("X (m)\tY (m)\tTheta (deg)");
}

// ---------------- LOOP ----------------
void loop() {
  long currentLeft = leftPulses;
  long currentRight = rightPulses;

  long deltaLeft = currentLeft - prevLeftPulses;
  long deltaRight = currentRight - prevRightPulses;

  prevLeftPulses = currentLeft;
  prevRightPulses = currentRight;

  float dLeft = (deltaLeft / pulsesPerRevLeft) * wheelCircumference;
  float dRight = (deltaRight / pulsesPerRevRight) * wheelCircumference;

  // --- Compute linear and angular displacement ---
  float dCenter = (dRight + dLeft) / 2.0;
  float dThetaRad = (dLeft - dRight) / wheelBase;   // radians
  float dThetaDeg = dThetaRad * (180.0 / PI);       // convert to degrees

  // --- Update global position (x, y, theta) ---
  float thetaRad = theta * (PI / 180.0);            // convert degrees to radians for cos/sin
  x += dCenter * cos(thetaRad + dThetaRad / 2.0);
  y += dCenter * sin(thetaRad + dThetaRad / 2.0);
   theta  += dThetaDeg;                               // update orientation in degrees

  // --- Normalize theta to 0–360 ---
  if (theta < 0) theta += 360;
  else if (theta >= 360) theta -= 360;

  // --- Print updated coordinates ---
  Serial.print("x = ");
  Serial.print(x, 3);
  Serial.print("\t");
  Serial.print("y = ");
  Serial.print(y, 3);
  Serial.print("\t");
  Serial.print("theta = ");
  Serial.println(theta, 2);

  delay(100);
}