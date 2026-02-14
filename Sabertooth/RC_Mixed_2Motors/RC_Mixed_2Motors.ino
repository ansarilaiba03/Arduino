//mixed
#include <Servo.h>

Servo motor1Input;  // S1
Servo motor2Input;  // S2

// Function to convert analog value (0-255) to microseconds for Sabertooth
int analogToMicro(int val) {
  return map(val, 0, 255, 1000, 2000);
}

void setup() {
  motor1Input.attach(9);   // S1
  motor2Input.attach(11);  // S2
}

void loop() {
   // Stop
  motor1Input.writeMicroseconds(analogToMicro(127)); // stop
  motor2Input.writeMicroseconds(analogToMicro(127)); // stop
  delay(2000);
  // --------- Forward ---------
  motor1Input.writeMicroseconds(analogToMicro(150)); // forward
  motor2Input.writeMicroseconds(analogToMicro(127)); // straight
  delay(5000);

  // --------- Backward ---------
  motor1Input.writeMicroseconds(analogToMicro(110));   // reverse
  motor2Input.writeMicroseconds(analogToMicro(127));  // straight
  delay(5000); 

  // --------- Turn Left ---------
  motor1Input.writeMicroseconds(analogToMicro(127)); // stop throttle
  motor2Input.writeMicroseconds(analogToMicro(110));  // left 
  delay(5000);

  // --------- Turn Right ---------
  motor1Input.writeMicroseconds(analogToMicro(127)); 
  motor2Input.writeMicroseconds(analogToMicro(150)); 
  delay(10000);
}