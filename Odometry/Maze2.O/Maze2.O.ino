#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// ---------------- ENCODER PINS ----------------
const int encoderLeftA = 2;  
const int encoderLeftB = 28; 
const int encoderRightA = 19; 
const int encoderRightB = 34;

// ---------------- PROXIMITY SENSOR ----------------
const int proxPin1 = 38;   // front sensor
const int proxPin2 = 42;   // side sensor

// ---------------- MOTOR PINS ----------------
const int motorLeftPWM = 6;
const int motorLeftDIR = 30;
const int motorRightPWM = 8;
const int motorRightDIR = 32;

// ---------------- ROBOT CONSTANTS ----------------
const float pulsesPerRevLeft = 400.0;
const float pulsesPerRevRight = 400.0;
const float wheelDiameter = 152.0;   // mm
const float wheelCircumference = PI * wheelDiameter;

// ---------------- ENCODER COUNTS ----------------
volatile long leftPulses = 0;
volatile long rightPulses = 0;
long prevLeftPulses = 0;
long prevRightPulses = 0;

// ---------------- POSITION VARIABLES ----------------
float x = 0.0;
float y = 0.0;
float theta = 0.0; // degrees

// ---------------- PID Constants ----------------
double Kp = 2.0;
double Ki = 0.0;
double Kd = 0.1;

// --- Separate PID constants for Yaw Control ---
double Kp_yaw = 1.1;
double Ki_yaw = 0.0;
double Kd_yaw = 1.0;

// ---------------- PID Variables ----------------
double integral = 0, lastError = 0;
unsigned long lastTime = 0;

// ---------------- IMU ----------------
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// ---------------- TARGET ----------------
const float targetX = 2000.0;  
const float targetY = 2000.0;  

// ---------------- STATE VARIABLES ----------------
bool bypassing = false;      // currently bypassing obstacle
bool movingSide = false;     // moving along side sensor path
float lastTurnYaw = 0;       // last yaw after turning

bool targetReached() {
  return (abs(x - targetX) < 100 && abs(y - targetY) < 100);
}

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
  // clockwise
  yawDeg = 360.0 - yawDeg;
  if (yawDeg >= 360) yawDeg -= 360;
  return yawDeg;
}

double computeDistancePID(double target, double current) {
  unsigned long now = millis();
  double dt = (now - lastTime) / 1000.0;
  if (dt <= 0) dt = 0.001;
  double error = target - current;
  integral += error * dt;
  double derivative = (error - lastError) / dt;
  double output = Kp * error + Ki * integral + Kd * derivative;
  lastError = error;
  lastTime = now;
  return constrain(output, -70, 70);
}

double computeYawPID(double setpoint, double input) {
  static double lastYawError = 0, yawIntegral = 0;
  static unsigned long lastYawTime = 0;
  unsigned long now = millis();
  double dt = (now - lastYawTime) / 1000.0;
  if (dt <= 0) dt = 0.001;
  double error = input - setpoint;
  if (error > 180) error -= 360;
  if (error < -180) error += 360;
  yawIntegral += error * dt;
  double derivative = (error - lastYawError) / dt;
  double output = Kp_yaw * error + Ki_yaw * yawIntegral + Kd_yaw * derivative;
  lastYawError = error;
  lastYawTime = now;
  return constrain(output, -70, 70);
}

bool hitBoundary() {
  return (x <= -2000 || x >= 2000 || y <= -2000 || y >= 2000);
}

bool frontObstacle() {
  return digitalRead(proxPin1) == LOW;
}

bool sideObstacle() {
  return digitalRead(proxPin2) == LOW;
}

// ---------------- DRIVE FUNCTIONS ----------------
void drive(double forwardPID) {
  double leftPWM = constrain(forwardPID, -70, 70);
  double rightPWM = constrain(forwardPID, -70, 70);

  if (leftPWM >= 0) { digitalWrite(motorLeftDIR, LOW); analogWrite(motorLeftPWM, leftPWM);}
  else { digitalWrite(motorLeftDIR, HIGH); analogWrite(motorLeftPWM, -leftPWM);}
  
  if (rightPWM >= 0) { digitalWrite(motorRightDIR, LOW); analogWrite(motorRightPWM, rightPWM);}
  else { digitalWrite(motorRightDIR, HIGH); analogWrite(motorRightPWM, -rightPWM);}
}

void stopMotors() {
  analogWrite(motorLeftPWM, 0);
  analogWrite(motorRightPWM, 0);
}

float calcTurn(float base, float add)
{
    float t = base + add;
    if (t >= 360) t -= 360;
    if (t < 0) t += 360;
    return t;
}

void rotateToYaw(float targetYaw) {
  while (true) {
    float currentYaw = getYaw();
    float error = currentYaw - targetYaw;
    if (error > 180) error -= 360;
    if (error < -180) error += 360;
    // PID for yaw
    double pidOut = computeYawPID(targetYaw, currentYaw);
    int pwm = constrain(abs(pidOut), 30, 80);
    if (pidOut > 0) {digitalWrite(motorLeftDIR, LOW); digitalWrite(motorRightDIR, HIGH);}
    else {digitalWrite(motorLeftDIR, HIGH); digitalWrite(motorRightDIR, LOW);}
    analogWrite(motorLeftPWM, pwm);
    analogWrite(motorRightPWM, pwm);
    if (abs(error) < 2) break;
  }
  stopMotors();
  delay(200);
}

void moveForwardDistanceSensor(int sensorPin) {
  while(digitalRead(sensorPin) == LOW) { // move while sensor sees red
    drive(50);
  }
  stopMotors();
}

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(9600);
  Wire.begin();
  if (!bno.begin()) {Serial.println("BNO055 not detected!"); while(1);}
  bno.setExtCrystalUse(true);
  delay(1000);

  pinMode(encoderLeftA, INPUT_PULLUP); pinMode(encoderLeftB, INPUT_PULLUP);
  pinMode(encoderRightA, INPUT_PULLUP); pinMode(encoderRightB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderLeftA), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderRightA), rightEncoderISR, RISING);

  pinMode(motorLeftPWM, OUTPUT); pinMode(motorLeftDIR, OUTPUT);
  pinMode(motorRightPWM, OUTPUT); pinMode(motorRightDIR, OUTPUT);
  pinMode(proxPin1, INPUT); pinMode(proxPin2, INPUT);

  lastTime = millis();
}

// ---------------- LOOP ----------------
void loop() {
  if (targetReached()) {
    stopMotors();
    Serial.println("TARGET REACHED! Robot stopped.");
    while(1);
  }

  // Update position from encoders
  long currentLeft = leftPulses;
  long currentRight = rightPulses;
  long deltaLeft = currentLeft - prevLeftPulses;
  long deltaRight = currentRight - prevRightPulses;
  prevLeftPulses = currentLeft;
  prevRightPulses = currentRight;

  float dLeft = (deltaLeft / pulsesPerRevLeft) * wheelCircumference;
  float dRight = (deltaRight / pulsesPerRevRight) * wheelCircumference;
  float dCenter = (dRight + dLeft) / 2.0;

  theta = getYaw();
  float thetaRad = radians(theta);
  x += dCenter * cos(thetaRad);
  y += dCenter * sin(thetaRad);

  // ---------------- MAIN LOGIC ----------------
  if (!bypassing && frontObstacle()) {
    // Obstacle detected → take right turn
    stopMotors();
    rotateToYaw( calcTurn(theta, 90) ); // right turn
    lastTurnYaw = getYaw();
    bypassing = true;
    movingSide = true;
  }

  else if (movingSide && sideObstacle()) {
    // Move along the bypass path until side sensor clears
    drive(50);
  }

  else if (movingSide && !sideObstacle()) {
    stopMotors();
    // After bypass, align to yaw = 0 (North)
    rotateToYaw(0);
    bypassing = false;
    movingSide = false;
  }

  // else if (hitBoundary()) {
  //   stopMotors();
  //   // If hit border, take left turn
  //  rotateToYaw( calcTurn(theta, -90) );
  // }

  else {
    // Normal forward motion
    drive(50);
  }

  // DEBUG
  Serial.print("X = "); Serial.print(x,1);
  Serial.print("  Y = "); Serial.print(y,1);
  Serial.print("  Theta = "); Serial.println(theta,1);
  delay(50);
}
 