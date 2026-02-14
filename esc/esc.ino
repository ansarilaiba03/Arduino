#include <Servo.h>
Servo esc;

int speedPWM = 1500;   // change this value (1000–2000)

void setup() {
  Serial.begin(9600);

  esc.attach(9);

  // Arm the ESC
  esc.writeMicroseconds(1000);
  delay(2000);

  Serial.println("Enter PWM value (1000–2000):");
}

void loop() {

  int potValue = analogRead(A8);
  // Clamp the value between 200 and 700
  if (potValue < 200) potValue = 200;
  if (potValue > 700) potValue = 700;
  int pwmValue = map(potValue, 200, 700, 1000, 2000);
  Serial.println(pwmValue);
  // Constant speed (your original logic)
  esc.writeMicroseconds(pwmValue);
}
