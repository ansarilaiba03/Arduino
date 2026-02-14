#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <math.h>

// ------------------- Motor Pins -------------------
const int RIGHT_DIR_PIN = 7;
const int RIGHT_PWM_CHANNEL = 9;   // PWM pin
const int LEFT_DIR_PIN = 8;
const int LEFT_PWM_CHANNEL = 10;  // PWM pin

// ------------------- Proximity Sensors -------------------
const int proxPin_200 = 2;
const int proxPin_400 = 4;
int obstacle = 0;
int previousObstacle = 0;  // track previous obstacle state

// ------------------- IMU -------------------
Adafruit_BNO055 bno = Adafruit_BNO055(55);

int motorSpeed = 30;

// ------------------- PID Constants -------------------
double Kp = 0.5;
double Ki = 0.0;
double Kd = 0.0;

// ------------------- Setup -------------------
void setup() {
  pinMode(LEFT_DIR_PIN, OUTPUT);
  pinMode(RIGHT_DIR_PIN, OUTPUT);
  pinMode(LEFT_PWM_CHANNEL, OUTPUT);
  pinMode(RIGHT_PWM_CHANNEL, OUTPUT);

  pinMode(proxPin_200, INPUT_PULLUP);
  pinMode(proxPin_400, INPUT_PULLUP);

  Serial.begin(9600);

  // Initialize IMU
  if (!bno.begin()) {
    Serial.println("BNO055 not detected!");
    while (1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);

  Serial.println("Setup complete");

  // Start moving forward immediately
  moveForward();
}

// ------------------- Motor Functions -------------------
void moveForward() {
  analogWrite(LEFT_PWM_CHANNEL, motorSpeed);
  analogWrite(RIGHT_PWM_CHANNEL, motorSpeed);
  digitalWrite(LEFT_DIR_PIN, LOW);
  digitalWrite(RIGHT_DIR_PIN, LOW);
}

void moveBackward() {
  analogWrite(LEFT_PWM_CHANNEL, motorSpeed);
  analogWrite(RIGHT_PWM_CHANNEL, motorSpeed);
  digitalWrite(LEFT_DIR_PIN, HIGH);
  digitalWrite(RIGHT_DIR_PIN, HIGH);
}

void moveRight() {
  analogWrite(LEFT_PWM_CHANNEL, motorSpeed);
  analogWrite(RIGHT_PWM_CHANNEL, motorSpeed);
  digitalWrite(LEFT_DIR_PIN, LOW);
  digitalWrite(RIGHT_DIR_PIN, HIGH);
}

void moveLeft() {
  analogWrite(LEFT_PWM_CHANNEL, motorSpeed);
  analogWrite(RIGHT_PWM_CHANNEL, motorSpeed);
  digitalWrite(LEFT_DIR_PIN, HIGH);
  digitalWrite(RIGHT_DIR_PIN, LOW);
}

void stopMotors() {
  analogWrite(LEFT_PWM_CHANNEL, 0);
  analogWrite(RIGHT_PWM_CHANNEL, 0);
  digitalWrite(LEFT_DIR_PIN, LOW);
  digitalWrite(RIGHT_DIR_PIN, LOW);
}

// ------------------- IMU Functions -------------------
double readYaw() {
  imu::Quaternion q = bno.getQuat();
  double yaw = atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                     1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
  yaw = yaw * 180.0 / M_PI;
  if (yaw < 0) yaw += 360.0;
  return yaw;
}

// ------------------- PID-based 90° Turns -------------------
void rotate90Right() {
  double targetYaw = readYaw() + 90;
  if (targetYaw >= 360) targetYaw -= 360;

  double error, prevError = 0, integral = 0;
  double output;

  while (true) {
    double currentYaw = readYaw();
    Serial.print("Yaw (Right Turn): "); Serial.println(currentYaw);

    // Compute shortest rotation error
    error = targetYaw - currentYaw;
    if (error > 180) error -= 360;
    if (error < -180) error += 360;

    // Stop turning if error small
    if (abs(error) < 1.0) break;

    // PID calculation
    integral += error;
    double derivative = error - prevError;
    output = Kp * error + Ki * integral + Kd * derivative;
    prevError = error;

    // Limit output to motorSpeed
    if (output > motorSpeed) output = motorSpeed;
    if (output < -motorSpeed) output = -motorSpeed;

    // Set motor directions and speed
    if (output > 0) { // turn right
      analogWrite(LEFT_PWM_CHANNEL, output);
      analogWrite(RIGHT_PWM_CHANNEL, output);
      digitalWrite(LEFT_DIR_PIN, HIGH);
      digitalWrite(RIGHT_DIR_PIN, LOW);
    } else { // turn left (if overshoot)
      analogWrite(LEFT_PWM_CHANNEL, -output);
      analogWrite(RIGHT_PWM_CHANNEL, -output);
      digitalWrite(LEFT_DIR_PIN, LOW);
      digitalWrite(RIGHT_DIR_PIN, HIGH);
    }
  }

  stopMotors();
}

void rotate90Left() {
  double targetYaw = readYaw() - 90;
  if (targetYaw < 0) targetYaw += 360;

  double error, prevError = 0, integral = 0;
  double output;

  while (true) {
    double currentYaw = readYaw();
    Serial.print("Yaw (Left Turn): "); Serial.println(currentYaw);

    // Compute shortest rotation error
    error = targetYaw - currentYaw;
    if (error > 180) error -= 360;
    if (error < -180) error += 360;

    // Stop turning if error small
    if (abs(error) < 1.0) break;

    // PID calculation
    integral += error;
    double derivative = error - prevError;
    output = Kp * error + Ki * integral + Kd * derivative;
    prevError = error;

    // Limit output to motorSpeed
    if (output > motorSpeed) output = motorSpeed;
    if (output < -motorSpeed) output = -motorSpeed;

    // Set motor directions and speed
    if (output > 0) { // turn right (if overshoot)
      analogWrite(LEFT_PWM_CHANNEL, output);
      analogWrite(RIGHT_PWM_CHANNEL, output);
      digitalWrite(LEFT_DIR_PIN, HIGH);
      digitalWrite(RIGHT_DIR_PIN, LOW);
    } else { // turn left
      analogWrite(LEFT_PWM_CHANNEL, -output);
      analogWrite(RIGHT_PWM_CHANNEL, -output);
      digitalWrite(LEFT_DIR_PIN, LOW);
      digitalWrite(RIGHT_DIR_PIN, HIGH);
    }
  }

  stopMotors();
}

// ------------------- Obstacle Detection -------------------
int checkObstacle() {
  int pinState_200 = digitalRead(proxPin_200);
  int pinState_400 = digitalRead(proxPin_400);

  if (pinState_200 == LOW && pinState_400 == HIGH) {
    Serial.println("Step detected");
    obstacle = 1;
  }
  else if (pinState_400 == LOW) {
    Serial.println("Scroll detected");
    obstacle = 1;
  }
  else if (pinState_200 == HIGH && pinState_400 == HIGH) {
    Serial.println("All clear!");
    obstacle = 0;
  }
  return obstacle;
}

// ------------------- Loop -------------------
void loop() {
  int flag = checkObstacle();

  if (flag) {
    // Stop first
    
    
    stopMotors();
    delay(500);

    // Turn right
    rotate90Right();
    delay(800);

    // Move forward a bit
    moveForward();
    delay(800);   // adjust distance
    stopMotors();
    delay(500);

    // Turn left to return to original heading
    rotate90Left();
    delay(800);

    // Start moving again
    moveForward();
  } 

  // Resume forward when obstacle removed
  if (!flag && previousObstacle == 1) {
    moveForward();
  }

  previousObstacle = flag;  // update previous state
}