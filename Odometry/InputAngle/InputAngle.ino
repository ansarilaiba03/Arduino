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
const int encoderLeftB = 13;
const int encoderRightA = 19;
const int encoderRightB = 34;

// ---------------- MOTOR PINS ----------------
const int motorLeftPWM = 6;
const int motorLeftDIR = 30;
const int motorRightPWM = 8;
const int motorRightDIR = 32;

// ---------------- ROBOT CONSTANTS ----------------
const float pulsesPerRevLeft = 400.0;
const float pulsesPerRevRight = 400.0;
const float wheelDiameter = 152.0;             // mm
const float wheelBase = 455.0;                 // mm
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
double Kp = 3.0;
double Ki = 0.0;
double Kd = 1.0;

// ---------------- PID Variables ----------------
double setpoint = 0;   // Target angle
double input = 0;      // Current yaw from IMU
double output = 0;     // PID output
double error, lastError = 0, integral = 0, derivative = 0;
unsigned long lastTime = 0;

// ---------------- IMU ----------------
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// ---------------- SERIAL INPUT ----------------
bool newTargetReceived = false;
String inputString = "";

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

  error = input - setpoint;  
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
  Serial.println("Enter target angle (0-359): ");
  lastTime = millis();
}

// ---------------- LOOP ----------------
void loop() {
  // ----------- Handle Serial Input -----------
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      setpoint = inputString.toFloat();
      if (setpoint < 0 || setpoint >= 360) {
        Serial.println("Invalid angle. Enter value between 0 and 359.");
      } else {
        newTargetReceived = true;
        Serial.print("New target angle: ");
        Serial.println(setpoint);
      }
      inputString = "";
    } else {
      inputString += c;
    }
  }

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

  input = getYaw();
  theta = input;

if (abs(dLeft - dRight) < 5) {  // threshold in mm, tweak if needed
  float thetaRad = theta * PI / 180.0;
  x += dCenter * cos(thetaRad);
  y += dCenter * sin(thetaRad);
}

  // ----------- PID Control -----------
  if (newTargetReceived) {
    double pidOut = computePID(setpoint, input);
    if (abs(error) < 2) {
      analogWrite(motorLeftPWM, 0);
      analogWrite(motorRightPWM, 0);
      Serial.println("Target reached.");
      Serial.println("Enter next target angle (0-359): ");
      newTargetReceived = false;
    } else {
      rotate(pidOut);
    }
  } else {
    analogWrite(motorLeftPWM, 0);
    analogWrite(motorRightPWM, 0);
  }

  // ----------- Debug Print -----------
  Serial.print("X = ");
  Serial.print(x / 1000.0, 3);  // Convert mm to meters
  Serial.print("\tY = ");
  Serial.print(y / 1000.0, 3);
  Serial.print("\tTheta = ");
  Serial.print(theta, 2);
  Serial.print("\tTarget = ");
  Serial.print(setpoint);
  Serial.print("\tError = ");
  Serial.println(error, 2);

  delay(100);
}
