#include <Wire.h>

const int motor1_dir = 4;
const int motor1_pwm = 5;
const int motor2_dir = 6;
const int motor2_pwm = 7;

int8_t lx = 0, ly = 0;

void setup() {
  pinMode(motor1_dir, OUTPUT);
  pinMode(motor1_pwm, OUTPUT);
  pinMode(motor2_dir, OUTPUT);
  pinMode(motor2_pwm, OUTPUT);

  Wire.begin(8);
  Wire.onReceive(receiveEvent);

  Serial.begin(9600);
}

void receiveEvent(int bytes) {
  if (Wire.available() >= 2) {
    lx = Wire.read();
    ly = Wire.read();
  }
}

void moveForward(int speed) {
  digitalWrite(motor1_dir, LOW);
  digitalWrite(motor2_dir, LOW);
  analogWrite(motor1_pwm, speed);
  analogWrite(motor2_pwm, speed);
}

void moveBackward(int speed) {
  digitalWrite(motor1_dir, HIGH);
  digitalWrite(motor2_dir, HIGH);
  analogWrite(motor1_pwm, speed);
  analogWrite(motor2_pwm, speed);
}

void turnLeft(int speed) {
  digitalWrite(motor1_dir, HIGH);
  digitalWrite(motor2_dir, LOW);
  analogWrite(motor1_pwm, speed);
  analogWrite(motor2_pwm, speed);
}

void turnRight(int speed) {
  digitalWrite(motor1_dir, LOW);
  digitalWrite(motor2_dir, HIGH);
  analogWrite(motor1_pwm, speed);
  analogWrite(motor2_pwm, speed);
}

void stopMotors() {
  analogWrite(motor1_pwm, 0);
  analogWrite(motor2_pwm, 0);
}

void loop() {
  int speed = map(abs(ly), 0, 127, 0, 255);

  if (ly < -20) {
    moveForward(speed);
  }
  else if (ly > 20) {
    moveBackward(speed);
  }
  else if (lx < -20) {
    turnLeft(150);
  }
  else if (lx > 20) {
    turnRight(150);
  }
  else {
    stopMotors();
  }

  delay(20);
}