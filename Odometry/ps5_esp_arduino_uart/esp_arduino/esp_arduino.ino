#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>

// ------------------- IMU -------------------
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);


// ------------------- Motor Pins -------------------
const int LEFT_PWM_PIN  = 6;
const int LEFT_DIR_PIN  = 26;
const int RIGHT_PWM_PIN = 8;
const int RIGHT_DIR_PIN = 24;

// ------------------- Encoder Pins -------------------
const int encoderLeftA  = 2;
const int encoderLeftB  = 13;
const int encoderRightA = 3;
const int encoderRightB = 12;

// ------------------- Robot Constants -------------------
const float pulsesPerRevLeft  = 400.0;
const float pulsesPerRevRight = 400.0;
const float wheelDiameter = 152.0;   // mm
const float wheelCircumference = 3.14159 * wheelDiameter;
const float wheelBase = 491.0;       // mm

// ------------------- Odometry -------------------
volatile long leftPulses = 0;
volatile long rightPulses = 0;
long prevLeftPulses = 0;
long prevRightPulses = 0;
float x = 0, y = 0, theta = 0;

// ------------------- Joystick -------------------
int ly = 0, rx = 0;
float speedFactor = 0.1;

// ------------------- Wheel Speeds -------------------
double targetwl = 0, targetwr = 0;

// ------------------- Encoder ISRs -------------------
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

// ------------------- IMU Yaw -------------------
double readYaw() {
  imu::Quaternion q = bno.getQuat();
  double yaw = atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                     1.0 - 2.0 * (q.y()*q.y() + q.z()*q.z()));
  yaw = yaw * 180.0 / 3.14159;
  if (yaw < 0) yaw += 360.0;
  return yaw;
}

// ------------------- Wheel Speed Logic -------------------
void compute2Wheel(int lyInput, int rxInput) {
  float V = -lyInput;  //  for forward/back
  float rX = rxInput;  // Turning from right stick

  targetwr = ((V * 200 + rX * wheelBase / 2) / (wheelDiameter / 2)); //angular velocity of right wheel
  targetwl = ((V * 200 - rX * wheelBase / 2) / (wheelDiameter / 2)); //angular velocity of left wheel

  targetwr = constrain(targetwr * speedFactor, -255, 255);
  targetwl = constrain(targetwl * speedFactor, -255, 255);
}

// ------------------- Apply PWM -------------------
void applySpeed() {
  if (targetwl >= 0) {
    digitalWrite(LEFT_DIR_PIN, LOW); //    low = forward
    analogWrite(LEFT_PWM_PIN, targetwl);
  } else {
    digitalWrite(LEFT_DIR_PIN, HIGH);
    analogWrite(LEFT_PWM_PIN, -targetwl);
  }

  if (targetwr >= 0) {
    digitalWrite(RIGHT_DIR_PIN, LOW);
    analogWrite(RIGHT_PWM_PIN, targetwr);
  } else {
    digitalWrite(RIGHT_DIR_PIN, HIGH);
    analogWrite(RIGHT_PWM_PIN, -targetwr);
  }
}

// ------------------- Setup -------------------
void setup() {
  Serial.begin(9600);    // Debug
  Serial1.begin(9600);   // UART from ESP32

  if (!bno.begin()) {
    Serial.println("BNO055 not detected!");
    while (1);
  }
  bno.setExtCrystalUse(true);
  delay(1000);

  pinMode(LEFT_PWM_PIN, OUTPUT);
  pinMode(LEFT_DIR_PIN, OUTPUT);
  pinMode(RIGHT_PWM_PIN, OUTPUT);
  pinMode(RIGHT_DIR_PIN, OUTPUT);

  pinMode(encoderLeftA, INPUT_PULLUP);
  pinMode(encoderLeftB, INPUT_PULLUP);
  pinMode(encoderRightA, INPUT_PULLUP);
  pinMode(encoderRightB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoderLeftA), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderRightA), rightEncoderISR, RISING);
}

// ------------------- Loop -------------------
void loop() {
  // ----------- Fast UART Receive -----------
  static char buffer[20];
  static int index = 0;

  while (Serial1.available()) {
    char c = Serial1.read();
    if (c == '\n') {
      buffer[index] = '\0';
      index = 0;

      char* comma = strchr(buffer, ',');
      if (comma) {
        *comma = '\0';
        ly = atoi(buffer);
        rx = atoi(comma + 1);

        Serial.print("LY: "); Serial.print(ly);
        Serial.print("  RX: "); Serial.println(rx);
      }
    } else if (index < sizeof(buffer) - 1) {
      buffer[index++] = c;
    }
  }

  // ----------- Deadband for joystick drift -----------
  if (abs(ly) < 10) ly = 0;
  if (abs(rx) < 10) rx = 0;

  compute2Wheel(ly, rx);
  applySpeed();

  // ----------- Odometry -----------
  long deltaLeft  = leftPulses  - prevLeftPulses;
  long deltaRight = rightPulses - prevRightPulses;
  prevLeftPulses  = leftPulses;
  prevRightPulses = rightPulses;

  float dLeft  = (deltaLeft  / pulsesPerRevLeft)  * wheelCircumference;
  float dRight = (deltaRight / pulsesPerRevRight) * wheelCircumference;
  float dCenter = (dRight + dLeft) / 2.0;

  theta = readYaw();
  float thetaRad = theta * 3.14159 / 180.0;

  x += dCenter * cos(thetaRad);
  y += dCenter * sin(thetaRad);

  Serial.print("X: "); Serial.print(x, 3);
  Serial.print(" mm\tY: "); Serial.print(y, 3);
  Serial.print(" mm\tTheta: "); Serial.println(theta, 2);

   // --- Reset if 'R' received from ESP (PS5 Circle button) ---
  if (Serial1.available()) {
    char c = Serial1.read();
    if (c == 'R') {
      x = 0;
      y = 0;
      theta = 0;
      leftPulses = 0;
      rightPulses = 0;
      prevLeftPulses = 0;
      prevRightPulses = 0;
      targetwl = 0;
      targetwr = 0;
      analogWrite(LEFT_PWM_PIN, 0);
      analogWrite(RIGHT_PWM_PIN, 0);
      Serial.println("Restart command received from PS5 — Motors and Odometry RESET!");
    }
  }
    // --- Reset if '1' sent over Serial Monitor ---
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '1') {
      x = 0;
      y = 0;
      theta = 0;
      leftPulses = 0;
      rightPulses = 0;
      prevLeftPulses = 0;
      prevRightPulses = 0;
      Serial.println("Odometry and encoder counts RESET!");
    }
  }



  delay(100);  // Short delay to improve responsiveness
}
