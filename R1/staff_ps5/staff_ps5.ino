#include <Arduino.h>
#include <ps5Controller.h>
#include <ESP32Servo.h>

// Servo objects
Servo servo1;
Servo servo2;

// Servo pins
const int servo1Pin = 2;
const int servo2Pin = 4;

// Initial angles
int angle1 = 90;
int angle2 = 90;

void setup() {
  Serial.begin(115200);

  // Attach servos
  servo1.attach(servo1Pin);
  servo2.attach(servo2Pin);

  // Set initial position
  servo1.write(angle1);
  servo2.write(angle2);

  // Start PS5 controller
  // ps5.begin("e8:47:3a:5a:a3:66");
  ps5.begin("7c:66:ef:78:76:f0");

  Serial.println("PS5 Controller Ready");
}

void loop() {

  // Check if controller connected
  if (ps5.isConnected()) {

    // SERVO 1 CONTROL
    // Square button -> increase angle
    if (ps5.Square()) {
      angle1 += 1;
      if (angle1 > 180) angle1 = 180;

      servo1.write(angle1);
      delay(10);
    }

    // Cross button -> decrease angle
    if (ps5.Cross()) {
      angle1 -= 1;
      if (angle1 < 0) angle1 = 0;

      servo1.write(angle1);
      delay(10);
    }

    // SERVO 2 CONTROL
    // Triangle button -> increase angle
    if (ps5.Triangle()) {
      angle2 += 1;
      if (angle2 > 180) angle2 = 180;

      servo2.write(angle2);
      delay(10);
    }

    // Circle button -> decrease angle
    if (ps5.Circle()) {
      angle2 -= 1;
      if (angle2 < 0) angle2 = 0;

      servo2.write(angle2);
      delay(10);
    }
  }
}