#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Create BNO055 object (default I2C address 0x28)
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("Initializing BNO055...");

  if (!bno.begin()) {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C address!");
    while (1);
  }

  delay(1000);

  // Optional: Use external crystal for better accuracy
  bno.setExtCrystalUse(true);

  Serial.println("Move the robot slowly in all directions to calibrate...");
  Serial.println("System, Gyro, Accel, Mag");
}

void loop() {
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;

  // Get calibration status
  bno.getCalibration(&system, &gyro, &accel, &mag);

  Serial.print("Calib Status -> ");
  Serial.print("Sys:");
  Serial.print(system);
  Serial.print(" Gyro:");
  Serial.print(gyro);
  Serial.print(" Accel:");
  Serial.print(accel);
  Serial.print(" Mag:");
  Serial.println(mag);

  // When fully calibrated (all = 3), print sensor offsets
  if (system == 3 && gyro == 3 && accel == 3 && mag == 3) {
    adafruit_bno055_offsets_t calibData;
    bno.getSensorOffsets(calibData);

    Serial.println("\nFully Calibrated! Save these offsets:");
    Serial.print("Accel Offset X: "); Serial.println(calibData.accel_offset_x);
    Serial.print("Accel Offset Y: "); Serial.println(calibData.accel_offset_y);
    Serial.print("Accel Offset Z: "); Serial.println(calibData.accel_offset_z);

    Serial.print("Gyro Offset X: "); Serial.println(calibData.gyro_offset_x);
    Serial.print("Gyro Offset Y: "); Serial.println(calibData.gyro_offset_y);
    Serial.print("Gyro Offset Z: "); Serial.println(calibData.gyro_offset_z);

    Serial.print("Mag Offset X: "); Serial.println(calibData.mag_offset_x);
    Serial.print("Mag Offset Y: "); Serial.println(calibData.mag_offset_y);
    Serial.print("Mag Offset Z: "); Serial.println(calibData.mag_offset_z);

    Serial.print("Accel Radius: "); Serial.println(calibData.accel_radius);
    Serial.print("Mag Radius: "); Serial.println(calibData.mag_radius);

    Serial.println("\nStore these values and later load them using setSensorOffsets().");

    while (1); // Stop here after calibration complete
  }

  delay(500);
}