#include <Servo.h>

Servo myServo;
Servo myservo1;

void setup() {
  myServo.attach(10);
  myservo1.attach(3);   // Servo signal connected to pin 9
  myservo1.write(0);
  delay(500);
  myServo.write(90);   // Move servo to 0 degrees (reset position)
} 

void loop() {
  // Nothing here
}