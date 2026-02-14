#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <math.h>
#include <utility/imumaths.h>

// ------------------- IMU -------------------
#define BNO055_SAMPLERATE_DELAY_MS 50
Adafruit_BNO055 myIMU = Adafruit_BNO055();

// Filtered roll
float rollF_old = 0;
float rollF_new = 0;

void setup() {
  Serial.begin(9600);
  myIMU.begin();
  delay(1000);
  myIMU.setExtCrystalUse(true);

  Serial.println("BNO055 Roll: Accel + Euler + Quaternion + Filtered");
}

void loop() {
  // ----------------- Accelerometer roll -----------------
  imu::Vector<3> acc = myIMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  float Ax = acc.x() / 9.81;
  float Ay = acc.y() / 9.81;
  float Az = acc.z() / 9.81;

  float roll_accel = atan2(Ay, Az) * 180.0 / M_PI;

  // ----------------- Euler roll -----------------
  imu::Vector<3> euler = myIMU.getVector(Adafruit_BNO055::VECTOR_EULER);
  float roll_euler = euler.z() * -1;  // X-axis → roll

  // ----------------- Quaternion roll -----------------
  imu::Quaternion q = myIMU.getQuat();
  float roll_quat = atan2(2.0 * (q.w() * q.x() + q.y() * q.z()),
                          1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y())) * 180.0 / M_PI;

  // ----------------- Filtered roll (Accel + Euler) -----------------
  float combined = 0.5 * (roll_accel + roll_euler); // average accel + Euler
  rollF_new = 0.95 * rollF_old + 0.05 * combined;
  rollF_old = rollF_new;

  // ----------------- Print -----------------
  Serial.print(roll_accel);
  Serial.print(",");
  Serial.print(roll_euler); 
  Serial.print(",");
  Serial.print(roll_quat);
  Serial.print(",");
  Serial.println(rollF_new);

  delay(BNO055_SAMPLERATE_DELAY_MS);
}