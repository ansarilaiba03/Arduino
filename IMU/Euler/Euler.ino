#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

Adafruit_BNO055 bno = Adafruit_BNO055();

void setup() {
  Serial.begin(9600);
  Wire.begin();

  if (!bno.begin()) {
    Serial.println("BNO055 not detected!");
    while (1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);
}

void loop() {
  // Get Euler angles
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  float roll  = euler.x();  // X-axis → Roll
  float pitch = euler.y();  // Y-axis → Pitch
  float yaw   = euler.z();  // Z-axis → Yaw

  Serial.print("Roll: "); //yaw
  Serial.print(roll, 2);
  Serial.print(" | Pitch: "); //roll
  Serial.print(pitch, 2);
  Serial.print(" | Yaw: "); //pitch
  Serial.println(yaw, 2);

  delay(100);
}