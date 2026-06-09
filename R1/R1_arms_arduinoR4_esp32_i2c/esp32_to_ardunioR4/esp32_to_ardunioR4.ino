#include <Wire.h>

#define SLAVE_ADDRESS 8

const int right_pulse_pin = 9;
const int right_dir_pin   = 8;

const int left_pulse_pin  = 7;
const int left_dir_pin    = 4;

char receivedCommand = 'S';

unsigned long lastStep = 0;
unsigned long stepDelay = 1000;

void receiveEvent(int howMany) {

  while (Wire.available()) {
    receivedCommand = Wire.read();
  }
}

void setup() {

  Serial.begin(115200);

  pinMode(right_pulse_pin, OUTPUT);
  pinMode(right_dir_pin, OUTPUT);

  pinMode(left_pulse_pin, OUTPUT);
  pinMode(left_dir_pin, OUTPUT);

  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveEvent);
}

void stepMotors(bool leftDir, bool rightDir) {

  digitalWrite(right_dir_pin, rightDir);
  digitalWrite(left_dir_pin, leftDir);

  digitalWrite(right_pulse_pin, HIGH);
  digitalWrite(left_pulse_pin, HIGH);

  delayMicroseconds(5);

  digitalWrite(right_pulse_pin, LOW);
  digitalWrite(left_pulse_pin, LOW);
}

void loop() {

  unsigned long currentTime = micros();

  if (currentTime - lastStep >= stepDelay) {

    switch (receivedCommand) {

      case 'F':

        stepMotors(LOW, HIGH);
        break;

      case 'B':

        stepMotors(HIGH, LOW);
        break;

      case 'L':

        digitalWrite(left_dir_pin, HIGH);

        digitalWrite(left_pulse_pin, HIGH);
        delayMicroseconds(5);
        digitalWrite(left_pulse_pin, LOW);

        break;

      case 'R':

        digitalWrite(left_dir_pin, LOW);

        digitalWrite(left_pulse_pin, HIGH);
        delayMicroseconds(5);
        digitalWrite(left_pulse_pin, LOW);

        break;

      case 'S':

        break;
    }

    lastStep = currentTime;
  }
}