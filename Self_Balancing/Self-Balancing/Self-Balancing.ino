#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <math.h>

// ------------------- IMU -------------------
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// ------------------- Calibration Offsets -------------------
adafruit_bno055_offsets_t calibrationOffsets = {
  -24, 35, -9,    // accel x,y,z
  -367, 277, -9,  // mag x,y,z
  -3, 0, -2,      // gyro x,y,z
  1000, 646       // accel & mag radius
};

// ------------------- PID Constants -------------------
double kp = 7.0;
double ki = 0.5;
double kd = 0.09;

double setpoint = -25.0;  // Target upright angle
double input, output;
double error, lastError;
double integral, derivative;
unsigned long lastTime;

// ------------------- Motor Pins (ESP32 + Cytron) -------------------
#define M1_PWM 27
#define M1_DIR 33
#define M2_PWM 18
#define M2_DIR 16

// ------------------- Motor Control -------------------
void driveMotors(int correction) {
  int speed = constrain(abs(correction), 0, 255);
  if (speed > 0) speed = map(speed, 0, 255, 60, 255);  // add base speed ~60

  if (correction > 0) {
    digitalWrite(M1_DIR, LOW);
    digitalWrite(M2_DIR, LOW);
  } else {
    digitalWrite(M1_DIR, HIGH);
    digitalWrite(M2_DIR, HIGH);
  }

  analogWrite(M1_PWM, speed);
  analogWrite(M2_PWM, speed);
}

// ------------------- Setup -------------------
void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!bno.begin()) {
    Serial.println("BNO055 not detected!");
    while (1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);
  bno.setSensorOffsets(calibrationOffsets);

  pinMode(M1_DIR, OUTPUT);
  pinMode(M2_DIR, OUTPUT);
  pinMode(M1_PWM, OUTPUT);
  pinMode(M2_PWM, OUTPUT);

  lastTime = millis();

  Serial.println("Self-balancing robot started...");
  Serial.println("To update PID: type kp=1.5 , ki=0.05 , kd=0.2 etc.");
}

// ------------------- Serial Input for PID -------------------
void checkSerialInput() {
  if (Serial.available()) {
    String inputStr = Serial.readStringUntil('\n');
    inputStr.trim();

    if (inputStr.startsWith("kp=")) {
      kp = inputStr.substring(3).toFloat();
      Serial.print("Updated kp = "); Serial.println(kp);
    } else if (inputStr.startsWith("ki=")) {
      ki = inputStr.substring(3).toFloat();
      Serial.print("Updated ki = "); Serial.println(ki);
    } else if (inputStr.startsWith("kd=")) {
      kd = inputStr.substring(3).toFloat();
      Serial.print("Updated kd = "); Serial.println(kd);
    }
  }
}

// ------------------- Loop -------------------
void loop() {
  // ----------- Read IMU Roll (forward/backward tilt) -----------
  imu::Quaternion q = bno.getQuat();
  double roll = atan2(2.0 * (q.w() * q.x() + q.y() * q.z()),
                      1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y()));
  roll = roll * 180.0 / M_PI;

  // // input = roll;
  // imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  // float Ax = accel.x();
  // float Ay = accel.y();
  // float Az = accel.z();

  // ----------- Calculate Roll & Pitch using Accelerometer -----------
  // double roll  = atan2(Ay, Az) * 180.0 / M_PI;                      // Roll
  // double pitch = atan2(-Ax, sqrt(Ay*Ay + Az*Az)) * 180.0 / M_PI;    // Pitch 

  input = roll;

  // ----------- PID Calculation -----------
  unsigned long now = millis();
  double dt = (now - lastTime) / 1000.0;
  if (dt <= 0) dt = 0.001;

  error = setpoint - input;
  integral += error * dt;
  derivative = (error - lastError) / dt;

  output = kp * error + ki * integral + kd * derivative;
  lastError = error;
  lastTime = now;

  int correction = (int)output;

  // ----------- Drive Motors -----------
  driveMotors(correction);

  // Debugging
  Serial.print("Roll: ");
  Serial.print(roll);
  Serial.print("  Output: ");
  Serial.print(output);
  Serial.print("  PID: kp=");
  Serial.print(kp, 6);
  Serial.print(" ki=");
  Serial.print(ki, 6);
  Serial.print(" kd=");
  Serial.println(kd, 6);

  checkSerialInput();  // <--- check for live PID updates

  delay(10);
}