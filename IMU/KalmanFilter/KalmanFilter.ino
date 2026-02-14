#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>

// ---------- CONFIG ----------
#define BNO055_SAMPLERATE_DELAY_MS 20
Adafruit_BNO055 myIMU = Adafruit_BNO055();

// Filtered roll
float rollF_old1 = 0;
float rollF_new1 = 0;
float rollF_old2 = 0;
float rollF_new2 = 0;

// ---------- Kalman Fusion Variables ----------
float kalmanAngle = 0.0f;  // fused roll
float P = 0.1f;            // error covariance (p> then less trust on kalmanAngle)
float R = 1.0f;            // measurement noise (tune this)
float K = 0.0f;            // Kalman gain (k> then less trust on kalmanAngle)

// weights for fusion of Euler + Accel
const float wEuler = 0.7;   // weight for Euler (stable)
const float wAccel = 0.3;   // weight for accelerometer (fast/noisy)

// timing
unsigned long lastTime = 0;

void setup() {
  Serial.begin(9600);
  if (!myIMU.begin()) {
    Serial.println("BNO055 not detected. Check wiring!");
    while (1);
  }
  delay(1000);
  myIMU.setExtCrystalUse(true);

  // initialize Kalman angle as weighted combination
  imu::Vector<3> accInit = myIMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  float roll_acc0 = atan2(accInit.y(), accInit.z()) * 180.0f / M_PI;
  
  imu::Vector<3> eulerInit = myIMU.getVector(Adafruit_BNO055::VECTOR_EULER);
  float roll_eul0 = eulerInit.z() * -1.0f;

  kalmanAngle = wEuler * roll_eul0 + wAccel * roll_acc0;

  lastTime = millis();

  Serial.println("roll_accel,roll_euler,roll_quat,roll_kalman");
}

void loop() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0f;
  if (dt <= 0.0f) dt = 0.001f;  // prevent division by zero
  lastTime = now;

  // -------- Accelerometer roll --------
  imu::Vector<3> acc = myIMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  float roll_accel = atan2(acc.y(), acc.z()) * 180.0f / M_PI;

  // -------- Euler roll --------
  imu::Vector<3> euler = myIMU.getVector(Adafruit_BNO055::VECTOR_EULER);
  float roll_euler = euler.z() * -1.0f;

  // -------- Quaternion roll (reference) --------
  imu::Quaternion q = myIMU.getQuat();
  float roll_quat = atan2(2.0f * (q.w() * q.x() + q.y() * q.z()),
                          1.0f - 2.0f * (q.x()*q.x() + q.y()*q.y())) * 180.0f / M_PI;

  // -------- Fusion measurement --------
  float measurement = wEuler * roll_euler + wAccel * roll_accel;

  // -------- Kalman update (1D) --------
  // Predict step: we assume no change in angle for now (no gyro prediction)
  // Update step
  K = P / (P + R);
  kalmanAngle = kalmanAngle + K * (measurement - kalmanAngle);
  P = (1 - K) * P + 0.01;  // small process noise to allow adaptation

  // ----------------- Filtered roll (Kalman + Euler) -----------------
  float combined1 = 0.5 * (kalmanAngle + roll_euler); // average accel + Euler
  rollF_new1 = 0.95 * rollF_old1 + 0.05 * combined1;
  rollF_old1 = 0.95 * rollF_old1 + 0.05 * combined1;
  rollF_old1 = rollF_new1;

  // ----------------- Filtered roll (Kalman + Quat) -----------------
  float combined2 = 0.5 * (kalmanAngle + roll_quat); // average accel + Euler
  rollF_new2 = 0.95 * rollF_old2 + 0.05 * combined2;
  rollF_old2 = 0.95 * rollF_old2 + 0.05 * combined2;
  rollF_old2 = rollF_new2;


  // -------- Print for Serial Plotter --------
  //Serial.print(roll_accel); Serial.print(",");
  //Serial.print(roll_euler); Serial.print(",");
  Serial.print(roll_quat); Serial.print(",");
  Serial.print(kalmanAngle); Serial.print(",");
  Serial.print(rollF_new1);Serial.print(",");
  Serial.println(rollF_new2);


  delay(BNO055_SAMPLERATE_DELAY_MS);
}