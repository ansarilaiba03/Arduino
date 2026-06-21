#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void setup()
{
  Serial.begin(115200);

  while (!Serial);

  Serial.println("Initializing BNO055...");

  if (!bno.begin())
  {
    Serial.println("BNO055 NOT FOUND!");
    while (1);
  }

  delay(1000);

  bno.setExtCrystalUse(true);

  Serial.println("BNO055 Ready!");
}

void loop()
{
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  Serial.print("Yaw: ");
  Serial.print(euler.x());

  Serial.print("   Roll: ");
  Serial.print(euler.z());

  Serial.print("   Pitch: ");
  Serial.println(euler.y());

  delay(100);
}