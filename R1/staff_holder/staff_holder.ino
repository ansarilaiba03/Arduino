#include <Servo.h>

Servo myservo;

String input = "";

void setup() {

  myservo.attach(9);

  Serial.begin(9600);

  Serial.println("Enter angle:");
}

void loop() {

  if (Serial.available()) {

    input = Serial.readStringUntil('\n');

    int angle = input.toInt();

    angle = constrain(angle, 0, 180);

    myservo.write(angle);

    Serial.print("Angle: ");
    Serial.println(angle);
  }
}