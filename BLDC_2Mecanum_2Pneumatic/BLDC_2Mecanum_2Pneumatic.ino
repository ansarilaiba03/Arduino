# include<Arduino.h>
# include<ps5Controller.h>
# include<ESP32Servo.h>

const float L_half = 275.0;
const float W_half = 262.5;
const float rad = 75.0;            
const float maxLinearSpeed = 100.0; 
const float maxOmega = 0.5;        
const float buffer = 0.0; 
const float deadZone = 10.0;

const int wfr_dir_pin = 14;
const int wfl_dir_pin = 12;
const int wrr_dir_pin = 18;
const int wrl_dir_pin = 19;
const int wfr_pwm_pin = 26;
const int wfl_pwm_pin = 25;
const int wrr_pwm_pin = 33;
const int wrl_pwm_pin = 32;
const int bldc_pin = 16;
// const int pnuematic_pwm_pin = 2;
const int pnuematic_dir_pin = 34;

unsigned long last_time = 0;
unsigned long bldc_delay = 10;

bool gotdata = false;
bool bldc_started = false;

float lx, ly, rx;
uint8_t wfr_pwm, wfl_pwm, wrr_pwm, wrl_pwm;
float wfr_speed, wfl_speed, wrr_speed, wrl_speed;
int up, down, square, circle, tri, cross;

Servo bldc;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  // ps5.begin("e8:47:3a:5a:a3:66");
  ps5.begin("7c:66:ef:78:76:f0");

  pinMode(wfr_dir_pin, OUTPUT);
  pinMode(wfl_dir_pin, OUTPUT);
  pinMode(wrr_dir_pin, OUTPUT);
  pinMode(wrl_dir_pin, OUTPUT);
  pinMode(wfr_pwm_pin, OUTPUT);
  pinMode(wfl_pwm_pin, OUTPUT);
  pinMode(wrr_pwm_pin, OUTPUT);
  pinMode(wrl_pwm_pin, OUTPUT);
  pinMode(bldc_pin, OUTPUT);
  // pinMode(pnuematic_pwm_pin, OUTPUT);
  pinMode(pnuematic_dir_pin, OUTPUT);


  bldc.attach(bldc_pin, 500, 2400);
  bldc.setPeriodHertz(50); 
  bldc.writeMicroseconds(1000);         
  delay(3000); 
}

void loop() {
  // put your main code here, to run repeatedly:

  if (ps5.isConnected()) {
    lx = ps5.LStickX();  
    ly = ps5.LStickY();
    rx = ps5.RStickX();
    up = ps5.Up();  
    down = ps5.Down();
    tri = ps5.Triangle();
    cross = ps5.Cross();
    square = ps5.Square();
    circle = ps5.Circle();

    gotdata = true;
  }

  static int speed = 1000;
  unsigned long start_time = micros();

  if (gotdata) {
    compute_wheel_speed();
    apply_wheel_speed();

    if (tri && (start_time - last_time >= bldc_delay) && !bldc_started) {
      speed = 1300;
      bldc.writeMicroseconds(speed);
      last_time = start_time;
      bldc_started = true;
      Serial.println("BLDC started");
    }

    if (circle && (start_time - last_time >= bldc_delay) && bldc_started && speed <= 2000) {
      speed += 2;
      bldc.writeMicroseconds(speed);
      last_time = start_time;
      Serial.println("BLDC increased");
      Serial.println(speed);
    }

    if (square && (start_time - last_time >= bldc_delay) && bldc_started && speed >= 999) {
      speed -= 2;
      bldc.writeMicroseconds(speed);
      last_time = start_time;
      Serial.println("BLDC decreased");
      Serial.println(speed);
    }

    if (cross && (start_time - last_time >= bldc_delay) && bldc_started) {
      speed = 1000;
      bldc.writeMicroseconds(speed);
      last_time = start_time;
      bldc_started = false;
      Serial.println("BLDC stopped");
    }

    if (up) {
      // digitalWrite(pnuematic_pwm_pin, HIGH);
      digitalWrite(pnuematic_dir_pin, HIGH);
    }

    if (down) {
      // digitalWrite(pnuematic_pwm_pin, LOW);
      digitalWrite(pnuematic_dir_pin, LOW);
    }
  }
}

void compute_wheel_speed() {
  
  if(abs(ly) < deadZone) ly = 0;
    else ly = (ly > 0) ? (ly-buffer) : (ly+buffer);

  if(abs(lx) < deadZone) lx = 0;
    else lx = (lx > 0) ? (lx-buffer) : (lx+buffer);

  if(abs(rx) < deadZone) rx = 0;
    else rx = (rx > 0) ? (rx-buffer) : (rx+buffer);

  float Vx = (lx/ 127.0f) * maxLinearSpeed;  
  float Vy = (ly/ 127.0f) * maxLinearSpeed;   
  float w = (rx/ 127.0f) * maxOmega; 

  // Serial.print("Vx: "); Serial.print(Vx);
  // Serial.print(" Vy: "); Serial.print(Vy);
  // Serial.print(" w: "); Serial.println(w);

  wfr_speed = ((Vy - Vx + w * (L_half + W_half)) / rad);
  wfl_speed = ((Vy - Vx - w * (L_half + W_half)) / rad);
  wrr_speed = ((Vy + Vx - w * (L_half + W_half)) / rad);
  wrl_speed = ((Vy + Vx + w * (L_half + W_half)) / rad);
}

void apply_wheel_speed() {

  digitalWrite(wfr_dir_pin, wfr_speed >= 0);
  digitalWrite(wfl_dir_pin, wfl_speed >= 0);
  digitalWrite(wrr_dir_pin, wrr_speed >= 0);
  digitalWrite(wrl_dir_pin, wrl_speed >= 0);

  wfr_pwm = constrain(abs(wfr_speed) * 78.1f, 0.0, 100.0);
  wfl_pwm = constrain(abs(wfl_speed) * 78.1f, 0.0, 100.0);
  wrr_pwm = constrain(abs(wrr_speed) * 78.1f, 0.0, 100.0);
  wrl_pwm = constrain(abs(wrl_speed) * 78.1f, 0.0, 100.0);

  // Serial.print("WFR: "); Serial.print(wfr_pwm);
  // Serial.print(" WFL: "); Serial.print(wfl_pwm);
  // Serial.print(" WRR: "); Serial.print(wrr_pwm);
  // Serial.print(" WRL: "); Serial.println(wrl_pwm);

  analogWrite(wfr_pwm_pin, wfr_pwm);
  analogWrite(wfl_pwm_pin, wfl_pwm);
  analogWrite(wrr_pwm_pin, wrr_pwm);
  analogWrite(wrl_pwm_pin, wrl_pwm);
}   