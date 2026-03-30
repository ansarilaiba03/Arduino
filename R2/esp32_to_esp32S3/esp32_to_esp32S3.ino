# include <Arduino.h>

const int right_dir_pin = 11;
const int right_pulse_pin = 12;
const int left_dir_pin = 9;
const int left_pulse_pin = 10;
const int pneumatic_pin = 7;

const int step_delay = 10000;
const int last_step = 0;

bool gotData = false;

uint8_t right, left, up, down, packet[4];

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, 47, 48);

  pinMode(right_dir_pin, OUTPUT);
  pinMode(right_pulse_pin, OUTPUT);
  pinMode(left_dir_pin, OUTPUT);
  pinMode(left_pulse_pin, OUTPUT);
  pinMode(pneumatic_pin, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long start_time = micros();

  if (Serial1.available() >= sizeof(packet)) {
    Serial1.readBytes((uint8_t*)packet, sizeof(packet));

    right = packet[0];
    left = packet[1];
    up = packet[2];
    down = packet[3];

    gotData = true;
  }

  if (gotData) {
    Serial.print(right), Serial.print(" "), Serial.print(left), Serial.print(" "), Serial.print(up), Serial.print(" "), Serial.println(down);

    if (right) {
      if (start_time - last_step >= step_delay) {
      digitalWrite(right_dir_pin, LOW);
      digitalWrite(right_pulse_pin, HIGH);
      delayMicroseconds(5);
      digitalWrite(right_pulse_pin, LOW);

      digitalWrite(left_dir_pin, HIGH);
      digitalWrite(left_pulse_pin, HIGH);
      delayMicroseconds(5);
      digitalWrite(left_pulse_pin, LOW);
      }
    }

    if (left) {
      if (start_time - last_step >= step_delay) {
      digitalWrite(right_dir_pin, HIGH);
      digitalWrite(right_pulse_pin, HIGH);
      delayMicroseconds(5);
      digitalWrite(right_pulse_pin, LOW);

      digitalWrite(left_dir_pin, LOW);
      digitalWrite(left_pulse_pin, HIGH);
      delayMicroseconds(5);
      digitalWrite(left_pulse_pin, LOW);
      }
    }

    if (down) {
      analogWrite(pneumatic_pin, 255);
    }

    if (up) {
      analogWrite(pneumatic_pin, 0);
    }
  }

  delay(10);
}