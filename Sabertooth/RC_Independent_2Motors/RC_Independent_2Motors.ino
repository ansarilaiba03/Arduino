//independent
#include <Servo.h>

Servo motor1;  
Servo motor2;  

void setup() {
  // Attach RC signals to pins
  motor1.attach(9);   // Pin 9 → S1 (Motor 1)
  motor2.attach(11);  // Pin 10 → S2 (Motor 2)

  // Initialize both motors stopped (1500µs pulse = neutral)
  motor1.writeMicroseconds(1500);
  motor2.writeMicroseconds(1500); 
  delay(1000);
}

void loop() {
  // Motor 1 forward (forward/backward)
  motor1.writeMicroseconds(2000);  
  // Motor 2 forward (left/right)
  motor2.writeMicroseconds(2000);
  delay(2000);
}


