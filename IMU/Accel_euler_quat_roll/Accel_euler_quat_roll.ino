#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <math.h>

// ------------------- IMU -------------------
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Initialize IMU
  if (!bno.begin()) {
    Serial.println("BNO055 not detected!");
    while (1);
  }

  delay(1000);
  bno.setExtCrystalUse(true);

  Serial.println("BNO055 Accelerometer Roll & Pitch Demo");
}

void loop() {
  // Get accelerometer data
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

  float Ax = accel.x();
  float Ay = accel.y();
  float Az = accel.z();

  // ----------- Compute Roll & Pitch from Accelerometer -----------
  float rollA  = atan2(Ay, Az) * 180.0 / M_PI;
  //float pitch = atan2(-Ax, sqrt(Ay * Ay + Az * Az)) * 180.0 / M_PI;

  imu::Quaternion q = bno.getQuat();
  double rollB = atan2(2.0 * (q.w() * q.x() + q.y() * q.z()),
                      1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y()));
  rollB = rollB * 180.0 / M_PI;

  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  float rollC  = euler.z()*-1;  // X-axis → Roll
  
  // ----------- Print Values -----------
  // Serial.print("Roll: ");
  // Serial.print(rollA);
  // Serial.print(",");
  // Serial.print(rollC);
  // Serial.print(",");
  // Serial.println(rollB);
  // Serial.print("Pitch: ");
  // Serial.println(pitch);
  Serial.println(Ax);
  Serial.println(Ay);
  Serial.println(Az);

  delay(100);
}