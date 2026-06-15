#include <Servo.h>

Servo esc;

const int ESC_PIN = 13;

void setup()
{
  Serial.begin(115200);

  esc.attach(ESC_PIN);

  Serial.println("Arming ESC...");

  // Minimum throttle for arming
  esc.writeMicroseconds(1000);

  delay(5000);   // wait 5 sec for ESC to arm

  Serial.println("Starting motor...");
}

void loop()
{
  // Speed
  esc.writeMicroseconds(1600);

  delay(1000);

  // Stop
  esc.writeMicroseconds(1000);

  delay(5000);
}