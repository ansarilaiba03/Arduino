#include <Servo.h>

Servo myServo;

void setup() {
  myServo.attach(9);
  myServo.write(90);   // Move servo to 0 degrees (reset position)
} 

void loop() {
  // Nothing here
}