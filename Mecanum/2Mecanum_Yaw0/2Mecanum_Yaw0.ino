#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <math.h>

// ------------------- IMU -------------------
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// ------------------- PID Constants -------------------
double Kp = 0.5;
double Ki = 0.01;
double Kd = 0.0;

double setpoint = 0.0;  // Target yaw (degrees)
double input, output;
double error, lastError;
double integral, derivative;
unsigned long lastTime;

// ------------------- Motor Pins (Arduino) -------------------
#define M1_STEP 3
#define M1_DIR  4
#define M2_STEP 5
#define M2_DIR  6

// ------------------- Rotate function -------------------
void rotateMotors(int correction) {
  if (abs(correction)<2) return;
  int steps = constrain(abs(correction) / 5, 1, 10 ); // max steps per loop

  if (correction < 0) {
    // Turn LEFT → left motor backward, right motor forward
    digitalWrite(M1_DIR, LOW);
    digitalWrite(M2_DIR, HIGH);
  } else if (correction > 0) {
    // Turn RIGHT → left motor forward, right motor backward
    digitalWrite(M1_DIR, HIGH);
    digitalWrite(M2_DIR, LOW);
  } else {
    return; // no correction
  }

  // Generate step pulses manually
  for (int i = 0; i < steps; i++) {
    digitalWrite(M1_STEP, HIGH);
    digitalWrite(M2_STEP, HIGH);
    delayMicroseconds(2500); // pulse width
    digitalWrite(M1_STEP, LOW);
    digitalWrite(M2_STEP, LOW);
    delayMicroseconds(2500);
  }
}

// ------------------- Setup -------------------
void setup() {
  Serial.begin(9600);

  // Initialize IMU
  if (!bno.begin()) {
    Serial.println("BNO055 not detected!");
    while (1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);

  // Motor pins
  pinMode(M1_STEP, OUTPUT);
  pinMode(M1_DIR, OUTPUT);
  pinMode(M2_STEP, OUTPUT);
  pinMode(M2_DIR, OUTPUT);

  lastTime = millis();
}

// ------------------- Loop -------------------
void loop() {
  // ----------- Read IMU Yaw -----------
  imu::Quaternion q = bno.getQuat();
  double yaw = atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                     1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
  yaw = yaw * 180.0 / M_PI;
  if (yaw < 0) yaw += 360.0;
  if (yaw > 180) yaw -= 360.0; // range -180 to +180

  Serial.print("Yaw: ");
  Serial.println(yaw,2);

  input = yaw;

  // ----------- PID -----------
  unsigned long now = millis();
  double dt = (now - lastTime) / 1000.0;
  if (dt <= 0) dt = 0.001;

  error = setpoint - input;
  integral += error * dt;
  derivative = (error - lastError) / dt;

  output = Kp * error + Ki * integral + Kd * derivative;
  lastError = error;
  lastTime = now;

  int correction = (int)output;

  // ----------- Rotate Motors -----------
  rotateMotors(correction);

  delay(20);
}
