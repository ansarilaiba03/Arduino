#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <math.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

double velocity = 0.0;
double distance = 0.0;
unsigned long lastTime = 0;

void setup() {
  Serial.begin(115200);
  if (!bno.begin()) {
    Serial.println("BNO055 not detected!");
    while (1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);
  Serial.println("Starting distance calculation...");
  lastTime = millis();
}

void loop() {
  // Get accelerometer data (in m/s^2)
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

  double ax = accel.x();

  // Calculate dt
  unsigned long now = millis();
  double dt = (now - lastTime) / 1000.0;  // seconds
  lastTime = now;

  // Integrate acceleration -> velocity
  velocity += ax * dt;

  // Integrate velocity -> distance
  distance += velocity * dt;

  // Print values
  Serial.print("Ax: "); Serial.print(ax, 2);
  Serial.print("  Vel: "); Serial.print(velocity, 2);
  Serial.print("  Dist: "); Serial.println(distance, 2);

  delay(50);
}