/*
connections mecanum
DIR1 = 4 
PWM1 = 5
DIR2 = 6
PWM2 = 7
GND = GND

IMU
VIN = 5V
GND = GND 
SDA = 20 (MEGA)
SCL = 21 (MEGA)

ENCODER
ALEFT = 2 
BLEFT = 3
ARIGHT = 19
BRIGHT = 18
*/

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
#include <utility/imumaths.h>

// ---------------- ENCODER PINS ----------------
const int encoderLeftA = 2;
const int encoderLeftB = 3;
const int encoderRightA = 13;
const int encoderRightB = 28;

// ---------------- MOTOR PINS ----------------
const int motorLeftPWM =6; // 7;
const int motorLeftDIR = 26;
const int motorRightPWM = 8; //5;
const int motorRightDIR = 24; //4;

// ---------------- ROBOT CONSTANTS ----------------
const float pulsesPerRevLeft = 400.0; //100.0;
const float pulsesPerRevRight = 400; //140.0;
const float wheelDiameter = 152.0;   // mm
const float wheelCircumference = PI * wheelDiameter;
const float wheelBase = 491.0; //325.0;       // mm

// ---------------- ENCODER COUNTS ----------------
volatile long leftPulses = 0;
volatile long rightPulses = 0;
long prevLeftPulses = 0;
long prevRightPulses = 0;

// ---------------- POSITION VARIABLES ----------------
float x = 0.0;    // mm
float y = 0.0;    // mm
float theta = 0.0; // degrees

// ---------------- PID Constants ----------------
double Kp = 0.5;
double Ki = 0.0;
double Kd = 0.0;

// ---------------- PID Variables ----------------
double setpoint = 0;  // target distance (mm)
double input = 0;     // current distance moved (mm) 
double output = 0;    // PID output (speed)
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

  error = setpoint - input;
  integral += error * dt;
  derivative = (error - lastError) / dt;

  double output = Kp * error + Ki * integral + Kd * derivative;
  lastError = error;
  lastTime = now;
  return output;
}

void drive(double pidOutput) {
  int pwm = constrain(abs(pidOutput), 0, 255);

  if (pidOutput < 0) {
    // Move Forward
    digitalWrite(motorLeftDIR, HIGH);
    digitalWrite(motorRightDIR, HIGH);
  } else {
    // Move Backward
    digitalWrite(motorLeftDIR, LOW);
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

  Serial.println("Straight Motion PID Started...");
  Serial.println("Enter target distance in mm (e.g. 500 for forward, -500 for backward): ");
  lastTime = millis();
}

// ---------------- LOOP ----------------
void loop() {
  // ----------- Handle Serial Input -----------
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      setpoint = inputString.toFloat();
      Serial.print("New target distance: ");
      Serial.print(setpoint);
      Serial.println(" mm");
      newTargetReceived = true;
      inputString = "";
    } else {
      inputString += c;
    }
  }

  // ----------- Update Encoders -----------
  long currentLeft = leftPulses;
  long currentRight = rightPulses;

  long deltaLeft = currentLeft - prevLeftPulses;
  long deltaRight = currentRight - prevRightPulses;

  prevLeftPulses = currentLeft;
  prevRightPulses = currentRight;

  float dLeft = (deltaLeft / pulsesPerRevLeft) * wheelCircumference;
  float dRight = (deltaRight / pulsesPerRevRight) * wheelCircumference;
  float dCenter = (dRight + dLeft) / 2.0;

  // ----------- Update IMU Heading -----------
  theta = getYaw();
  float thetaRad = radians(theta);

  // ----------- Update x and y Position -----------
  x += dCenter * cos(thetaRad);
  y += dCenter * sin(thetaRad);
  input += dCenter;  // for distance PID

  // ----------- PID Control -----------
  if (newTargetReceived) {
    double pidOut = computePID(setpoint, input);
    if (abs(error) < 5) { // within 5 mm
      analogWrite(motorLeftPWM, 0);
      analogWrite(motorRightPWM, 0);
      Serial.println("Target distance reached.");
      Serial.println("Enter next target (mm): ");
      newTargetReceived = false;
      input = 0;
      integral = 0;
      lastError = 0;
    } else {
      drive(pidOut);
    }
  } else {
    analogWrite(motorLeftPWM, 0);
    analogWrite(motorRightPWM, 0);
  }

  // ----------- Debug Print -----------
  Serial.print("x = ");
  Serial.print(x, 1);
  Serial.print(" mm\t y = ");
  Serial.print(y, 1);
  Serial.print(" mm\t θ = ");
  Serial.print(theta, 2);
  Serial.print("°\t Distance = ");
  Serial.print(input, 1);
  Serial.print(" mm\t Error = ");
  Serial.println(error, 1);

  delay(100);
}