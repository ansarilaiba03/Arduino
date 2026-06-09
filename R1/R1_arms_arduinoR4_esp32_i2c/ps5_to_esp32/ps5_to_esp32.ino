#include <Arduino.h>
#include <Wire.h>
#include <ps5Controller.h>
#include <ESP32Servo.h>

uint8_t cross, squ, tri, circle, l1, r1;

const int right_servo_pin = 2;
const int left_servo_pin = 4;

int angle_right = 90;
int angle_left = 90;

Servo servoright;
Servo servoleft;

char command = 'S';

void setup() {

  Serial.begin(115200);

  //ps5.begin("e8:47:3a:5a:a3:66");
  ps5.begin("7c:66:ef:78:76:f0");

  Wire.begin(21, 22); // SDA, SCL

  servoright.attach(right_servo_pin, 500, 2400);
  servoleft.attach(left_servo_pin, 500, 2400);

  servoright.write(angle_right);
  servoleft.write(angle_left);
}

void sendCommand(char cmd) {

  Wire.beginTransmission(8); // UNO address
  Wire.write(cmd);
  Wire.endTransmission();
}

void loop() {

  if (ps5.isConnected()) {

    cross  = ps5.Cross();
    squ    = ps5.Square();
    tri    = ps5.Triangle();
    circle = ps5.Circle();
    l1     = ps5.L1();
    r1     = ps5.R1();

    // MOTOR COMMANDS

    if (tri) {
      sendCommand('F');
      Serial.print("up ");

    }

    else if (cross) {
      sendCommand('B');
      Serial.print("down ");

    }

    else if (squ) {
      sendCommand('L');
      Serial.print("open ");

    }

    else if (circle) {
      sendCommand('R');
      Serial.print("close ");

    }

    else {
      sendCommand('S');
    }

    // SERVO CONTROL

    if (l1) {

      angle_right++;
      angle_left--;

      angle_right = constrain(angle_right, 0, 180);
      angle_left  = constrain(angle_left, 0, 180);

      servoright.write(angle_right);
      servoleft.write(angle_left);

      Serial.print("left servo ");


      delay(20);
    }

    if (r1) {

      angle_right--;
      angle_left++;

      angle_right = constrain(angle_right, 0, 180);
      angle_left  = constrain(angle_left, 0, 180);

      servoright.write(angle_right);
      servoleft.write(angle_left);

      Serial.print("Right servo ");

      delay(20);
    }
  }
}