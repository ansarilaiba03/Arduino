# include<Arduino.h>
# include<ps5Controller.h>
# include<ESP32Servo.h>

uint8_t cross, squ, tri, circle, l1, r1;
uint8_t prev_l1 = 0;
uint8_t prev_r1 = 0;
 
const int right_pulse_pin = 19;
const int right_dir_pin = 18;
const int left_pulse_pin = 17;
const int left_dir_pin = 16;
const int right_servo_pin = 2;
const int left_servo_pin = 4;

unsigned long last_step = 0;
unsigned long step_delay = 1000;
 

int angle_right = 90;
int angle_left = 90;

bool gotData = false;

Servo servoright;
Servo servoleft;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  ps5.begin("e8:47:3a:5a:a3:66");
  // ps5.begin("7c:66:ef:78:76:f0");
  pinMode(right_pulse_pin, OUTPUT);
  pinMode(right_dir_pin, OUTPUT);
  pinMode(left_pulse_pin, OUTPUT);
  pinMode(left_dir_pin, OUTPUT);
  servoright.attach(right_servo_pin, 500, 2400);
  servoleft.attach(left_servo_pin, 500, 2400); 
  servoright.write(angle_right);
  servoleft.write(angle_left);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (ps5.isConnected()) {
    cross = ps5.Cross();  
    squ = ps5.Square();
    tri = ps5.Triangle();
    circle = ps5.Circle();
    l1 = ps5.L1();
    r1 = ps5.R1();

    gotData = true;
  }

  unsigned long start_time = micros();

  if (gotData) {
    if (tri && (start_time - last_step >= step_delay)) {
      digitalWrite(right_dir_pin, HIGH);
      digitalWrite(right_pulse_pin, LOW);
      delayMicroseconds(3);
      digitalWrite(right_pulse_pin, HIGH);

      digitalWrite(left_dir_pin, LOW);
      digitalWrite(left_pulse_pin, LOW);
      delayMicroseconds(3);
      digitalWrite(left_pulse_pin, HIGH);
      last_step = start_time;
    }

    if (cross && (start_time - last_step >= step_delay)) {
      digitalWrite(right_dir_pin, LOW);
      digitalWrite(right_pulse_pin, LOW);
      delayMicroseconds(3);
      digitalWrite(right_pulse_pin, HIGH);

      digitalWrite(left_dir_pin, HIGH);
      digitalWrite(left_pulse_pin, LOW);
      delayMicroseconds(3);
      digitalWrite(left_pulse_pin, HIGH);
      last_step = start_time;
    }

    if (squ && (start_time- last_step >= step_delay)) {
      digitalWrite(left_dir_pin, HIGH);
      digitalWrite(left_pulse_pin, HIGH);
      delayMicroseconds(5);
      digitalWrite(left_pulse_pin, LOW);
      last_step = start_time;
    }

    if (circle && (start_time - last_step >= step_delay)) {
      digitalWrite(left_dir_pin, LOW);
      digitalWrite(left_pulse_pin, HIGH);
      delayMicroseconds(5);
      digitalWrite(left_pulse_pin, LOW);
      last_step = start_time;
    }

    bool l1_pressed = ((l1 && !prev_l1) || (l1 && prev_l1)); 
    bool r1_pressed = (r1 && !prev_r1) || (r1 && prev_r1);

    if (l1_pressed) {
      angle_right += 1;
      angle_left  -= 1;
      angle_right = constrain(angle_right, 0, 180);
      angle_left  = constrain(angle_left,  0, 180);
      servoright.write(angle_right);
      servoleft.write(angle_left);
      delay(20);
    }

    if (r1_pressed) {
      angle_right -= 1;
      angle_left  += 1;
      angle_right = constrain(angle_right, 0, 180);
      angle_left  = constrain(angle_left,  0, 180);
      servoright.write(angle_right);
      servoleft.write(angle_left);
      delay(20);
    }

    prev_l1 = l1;
    prev_r1 = r1;
  }
}