/*
connections
DIR1 = 4
PWM1 = 5
DIR2 = 6
PWM2 = 7
GND = GND

IMU
VIN = 5V
GND = GND 
SDA = 20(MEGA)
SCL = 21 (MEGA)

ENCODER
ALEFT = 2 
BLEFT = 3
ARIGHT = 19
BRIGHT = 18 

*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// ---------------- ENCODER PINS ----------------
const int encoderLeftA = 2;
const int encoderLeftB = 3;
const int encoderRightA = 18;
const int encoderRightB = 19;

// ---------------- MOTOR PINS ----------------
const int motorLeftPWM = 7;
const int motorLeftDIR = 6;
const int motorRightPWM = 5;
const int motorRightDIR = 4;

// ---------------- ROBOT CONSTANTS ----------------
const float pulsesPerRevLeft = 100.0;
const float pulsesPerRevRight = 140.0;
const float wheelDiameter = 152.0;             // mm
const float wheelBase = 325.0;                 // mm
const float wheelCircumference = PI * wheelDiameter;

// ---------------- ENCODER COUNTS ----------------
volatile long leftPulses = 0;
volatile long rightPulses = 0;
long prevLeftPulses = 0;
long prevRightPulses = 0;

// ---------------- POSITION VARIABLES ----------------
float x = 0.0;     // meters
float y = 0.0;     // meters
float theta = 0.0; // degrees (orientation)

// ---------------- PID Constants ----------------
double Kp = 0.9;
double Ki = 0.0;
double Kd = 0.0;

// ---------------- PID Variables ----------------
double setpoint = 90;   // Target angle
double input = 0;      // Current yaw from IMU
double output = 0;     // PID output
double error, lastError = 0, integral = 0, derivative = 0;
unsigned long lastTime = 0;

// ---------------- IMU ----------------
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// ---------------- INTERRUPTS ----------------
void leftEncoderISR() {
  int A = digitalRead(encoderLeftA);
  int B = digitalRead(encoderLeftB);
  if (A == B) leftPulses++;
  else leftPulses--;
}

void rightEncoderISR() {
  int A = digitalRead(encoderRightA);
  int B = digitalRead(encoderRightB);
  if (A == B) rightPulses++;
  else rightPulses--;
}

// ---------------- HELPER FUNCTIONS ----------------
float getYaw() {
  imu::Quaternion quat = bno.getQuat();
  float qw = quat.w(), qx = quat.x(), qy = quat.y(), qz = quat.z();

  float yawRad = atan2(2.0 * (qw * qz + qx * qy),
                       1.0 - 2.0 * (qy * qy + qz * qz));
  float yawDeg = yawRad * 180.0 / PI;
  if (yawDeg < 0) yawDeg += 360.0;
  return yawDeg;
}

double computePID(double setpoint, double input) {
  unsigned long now = millis();
  double dt = (now - lastTime) / 1000.0;
  if (dt <= 0) dt = 0.001;

  error = setpoint - input;
  // Wrap-around fix
  if (error > 180) error -= 360;
  if (error < -180) error += 360;

  integral += error * dt;
  derivative = (error - lastError) / dt;

  double output = Kp * error + Ki * integral + Kd * derivative;
  lastError = error;
  lastTime = now;
  return output;
}

void rotate(double pidOutput) {
  int pwm = constrain(abs(pidOutput), 0, 255);

  if (pidOutput > 0) {
    // Turn Left
    digitalWrite(motorLeftDIR, LOW);
    digitalWrite(motorRightDIR, HIGH);
  } else {
    // Turn Right
    digitalWrite(motorLeftDIR, HIGH);
    digitalWrite(motorRightDIR, LOW);
  }

  analogWrite(motorLeftPWM, pwm);
  analogWrite(motorRightPWM, pwm);
}

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(9600);
  Wire.begin();

  // IMU setup
  if (!bno.begin()) {
    Serial.println("BNO055 not detected!");
    while (1);
  }
  bno.setExtCrystalUse(true);
  delay(1000);

  // Encoder setup
  pinMode(encoderLeftA, INPUT_PULLUP);
  pinMode(encoderLeftB, INPUT_PULLUP);
  pinMode(encoderRightA, INPUT_PULLUP);
  pinMode(encoderRightB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoderLeftA), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderRightA), rightEncoderISR, RISING);

  // Motor setup
  pinMode(motorLeftPWM, OUTPUT);
  pinMode(motorLeftDIR, OUTPUT);
  pinMode(motorRightPWM, OUTPUT);
  pinMode(motorRightDIR, OUTPUT);

  Serial.println("Odometry + IMU + PID Rotation Started...");
  Serial.println("X (m)\tY (m)\tTheta (deg)");
  lastTime = millis();
}

// ---------------- LOOP ----------------
void loop() {
  static int stage = 0; // 0 = left 90°, 1 = back to 0°, 2 = right 90°, 3 = stop

  // ----------- Update Encoders & Position -----------
  long currentLeft = leftPulses;
  long currentRight = rightPulses;

  long deltaLeft = currentLeft - prevLeftPulses;
  long deltaRight = currentRight - prevRightPulses;

  prevLeftPulses = currentLeft;
  prevRightPulses = currentRight;

  float dLeft = (deltaLeft / pulsesPerRevLeft) * wheelCircumference;
  float dRight = (deltaRight / pulsesPerRevRight) * wheelCircumference;

  float dCenter = (dRight + dLeft) / 2.0;

  // ----------- IMU Orientation -----------
  input = getYaw();
  theta = input;

// Only update position if robot is moving mostly forward/backward
if (abs(dLeft - dRight) < 5) {  // threshold in mm, tweak if needed
  float thetaRad = theta * PI / 180.0;
  x += dCenter * cos(thetaRad);
  y += dCenter * sin(thetaRad);
}

  // ----------- PID Control for Rotation -----------
  double pidOut = computePID(setpoint, input);
  rotate(pidOut);

  // ----------- Print Position & Orientation -----------
  Serial.print("X = ");
  Serial.print(x, 3);
  Serial.print("\tY = ");
  Serial.print(y, 3);
  Serial.print("\tTheta = ");
  Serial.print(theta, 2);
  Serial.print("\tTarget = ");
  Serial.print(setpoint);
  Serial.print("\tError = ");
  Serial.println(error, 2);

  // ----------- Stage Transitions -----------
  if (abs(error) < 2) {
    analogWrite(motorLeftPWM, 0);
    analogWrite(motorRightPWM, 0);
    delay(1000);

    if (stage == 0) {
      setpoint = 0;
      stage = 1;
    } else if (stage == 1) {
      setpoint = 270;  // equivalent to -90
      stage = 2;
    } else if (stage == 2) {
      Serial.println(" All rotations complete.");
      analogWrite(motorLeftPWM, 0);
      analogWrite(motorRightPWM, 0);
      while (1);
    }
  }

  delay(100);
}