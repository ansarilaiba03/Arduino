// =====================================
// ARDUINO UNO - BNO055 IMU
// I2C: A4 (SDA), A5 (SCL)
// UART TX to Mega: Pin 1 (TX)
// Baud: 115200
// Sends yaw as quaternion-derived float
// =====================================

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void setup()
{
  Serial.begin(115200);   // UART to Mega (pins 0 RX, 1 TX)

  if (!bno.begin())
  {
    // Halt if IMU not found
    while (1);
  }

  bno.setExtCrystalUse(true);
  delay(1000);
}

void loop()
{
  // Get quaternion from BNO055
  imu::Quaternion quat = bno.getQuat();

  double w = quat.w();
  double x = quat.x();
  double y = quat.y();
  double z = quat.z();

  // Derive yaw (rotation about Z axis) from quaternion
  // yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
  double sinYaw = 2.0 * (w * z + x * y);
  double cosYaw = 1.0 - 2.0 * (y * y + z * z);
  double yawRad = atan2(sinYaw, cosYaw);

  // Convert to degrees
  float yawDeg = (float)(yawRad * 180.0 / PI);

  // Send as "Y:<value>\n" over UART to Mega
  Serial.print("Y:");
  Serial.println(yawDeg, 2);

  delay(20);   // 50 Hz update rate
}