/*
connections omni
encoder 1
red 5v
black gnd
white -  2
green - 4

encoder 2
red - 5v
black - gnd
green - 3
white - 5

PWM1 - 6
DIR1 - 7
PWM2 - 8
DIR2 - 9
*/ 
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h> // Needed for quaternion math

// ---------------- ENCODER PINS ----------------
const int encoderLeftA = 2;
const int encoderLeftB = 4; //3;
const int encoderRightA = 3; //18;
const int encoderRightB = 5;  //19;

// ---------------- ROBOT CONSTANTS ----------------
const float pulsesPerRevLeft = 600.0;        //100.0;
const float pulsesPerRevRight = 400.0;       // 140.0;
const float wheelDiameter = 152.0;       // mm
const float wheelBase = 491.0;          //325.0;  // mm
const float wheelCircumference = PI * wheelDiameter;  // mm

// ---------------- ENCODER COUNTS ----------------
volatile long leftPulses = 0;
volatile long rightPulses = 0;

// ---------------- POSITION VARIABLES ----------------
float x = 0.0;     // meters
float y = 0.0;     // meters
float theta = 0.0; // degrees (orientation)

// Previous encoder pulses
long prevLeftPulses = 0;
long prevRightPulses = 0;

// ---------------- BNO055 IMU ----------------
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// ---------------- WORKING ENCODER ISR ----------------
void leftEncoderISR() {
  int A = digitalRead(encoderLeftA);
  int B = digitalRead(encoderLeftB);

  if (A == B)
    leftPulses++;
  else
    leftPulses--;
}

void rightEncoderISR() {
  int A = digitalRead(encoderRightA);
  int B = digitalRead(encoderRightB);

  if (A == B)
    rightPulses++;
  else
    rightPulses--;
}

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(9600);
  Wire.begin();

  if (!bno.begin()) {
    Serial.println("Failed to initialize BNO055!");
    while (1);
  }

  delay(1000);
  bno.setExtCrystalUse(true);

  pinMode(encoderLeftA, INPUT_PULLUP);
  pinMode(encoderLeftB, INPUT_PULLUP);
  pinMode(encoderRightA, INPUT_PULLUP);
  pinMode(encoderRightB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoderLeftA), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderRightA), rightEncoderISR, RISING);

  Serial.println("Odometry + BNO055 Quaternion IMU Started...");
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

  // Linear displacement (mm)
  float dCenter = (dRight + dLeft) / 2.0;

  // --------- Read Quaternion and Convert to Yaw ----------
  imu::Quaternion quat = bno.getQuat();

  float qw = quat.w();
  float qx = quat.x();
  float qy = quat.y();
  float qz = quat.z();

  // Yaw in radians
  float yawRad = atan2(2.0 * (qw * qz + qx * qy),
                       1.0 - 2.0 * (qy * qy + qz * qz));

  // Convert to degrees
  float yawDeg = yawRad * (180.0 / PI);

  // Normalize yaw to [0, 360)
  if (yawDeg < 0) yawDeg += 360.0;

  theta = yawDeg;
  

  // Update position (mm)
  if (abs(dLeft - dRight) < 5) {  // threshold in mm, tweak if needed
  float thetaRad = theta * PI / 180.0;   // Convert theta to radians for position update
  x += dCenter * cos(thetaRad);
  y += dCenter * sin(thetaRad);
}


  // Print position and orientation
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