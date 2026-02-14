#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>

#define SDA_PIN 10
#define SCL_PIN 11

// IMU 
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// Proximity Sensor
#define PROX_PIN 18
bool boxDetected = false;
const float BOX_OFFSET = 430.0; // mm (5 cm)

// Motor pins
const int LEFT_DIR_PIN = 6;
const int LEFT_PWM_PIN = 7;
const int RIGHT_DIR_PIN = 12;
const int RIGHT_PWM_PIN = 9;

// Encoder pins
const int encoderLeftA = 4; // white
const int encoderLeftB = 5; // green
const int encoderRightA = 3; // white
const int encoderRightB = 2; // green

// Constants 
const float pulsesPerRevLeft = 400.0;
const float pulsesPerRevRight = 400.0;
const float wheelDiameter = 152.0; //mm
const float wheelCircumference = 3.14 * wheelDiameter;
const float wheelBase = 455.0; //mm

// Odometry 
volatile long leftPulses = 0;
volatile long rightPulses = 0;
long prevLeftPulses = 0;
long prevRightPulses = 0;
float x = 0 , y = 0 , theta = 0;

// Joystick
int ly = 0, rx = 0;
float speedFactor = 0.2;

// Wheel Speed 
double targetwl= 0, targetwr = 0;

// Encoder ISRs
void leftEncoderISR(){
  int A = digitalRead(encoderLeftA);
  int B = digitalRead(encoderLeftB);
  if (A == B) leftPulses++;
  else leftPulses--;
}

void rightEncoderISR(){
  int A = digitalRead(encoderRightA);
  int B = digitalRead(encoderRightB);
  if (A == B) rightPulses++;
  else rightPulses--;
}

// IMU Yaw
double readYaw() {
  imu::Quaternion q = bno.getQuat();
  double yaw = atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                     1.0 - 2.0 * (q.y()*q.y() + q.z()*q.z()));
  yaw = yaw * 180.0 / 3.14159;
  if (yaw < 0) yaw += 360.0;
  return yaw;
}


// Wheel Speed Logic 
void compute2Wheel(int lyInput, int rxInput){
  float V = -lyInput; // forward/backward
  float rx = rxInput; // right/left

  targetwr = ((V * 200 + rx * wheelBase / 2) / (wheelDiameter/ 2)); // angular velocity for right wheel
  targetwl = ((V * 200 - rx * wheelBase / 2) / (wheelDiameter/ 2)); // angular velocity for left wheel

  targetwr = constrain(targetwr * speedFactor, -200, 200);
  targetwl = constrain(targetwl * speedFactor , -255, 255);
}

// Apply PWM
void applySpeed(){
  if (targetwl >= 0){
    digitalWrite(LEFT_DIR_PIN, HIGH);
    analogWrite(LEFT_PWM_PIN, targetwl);
  } else {
    digitalWrite(LEFT_DIR_PIN, LOW);
    analogWrite(LEFT_PWM_PIN, -targetwl);
  }

  if(targetwr >= 0){
    digitalWrite(RIGHT_DIR_PIN, HIGH);
    analogWrite(RIGHT_PWM_PIN, targetwr);
  } else {
    digitalWrite(RIGHT_DIR_PIN, LOW);
    analogWrite(RIGHT_PWM_PIN, -targetwr);
  }
}

// Setup
void setup() {
  Serial.begin(9600); //Debug
  Serial1.begin(9600, SERIAL_8N1, 47, 48); // RX=17, TX=16 (from ESP32)

  Wire.begin(SDA_PIN, SCL_PIN);
   if (!bno.begin()) {
    Serial.println("BNO055 not detected!");
    while (1);
  }
  bno.setExtCrystalUse(true);
  delay(1000);

  pinMode(PROX_PIN, INPUT_PULLUP);

  pinMode(LEFT_DIR_PIN, OUTPUT);
  pinMode(LEFT_PWM_PIN, OUTPUT);
  pinMode(RIGHT_DIR_PIN, OUTPUT);
  pinMode(RIGHT_PWM_PIN, OUTPUT);

  pinMode(encoderLeftA, INPUT_PULLUP);
  pinMode(encoderLeftB, INPUT_PULLUP);
  pinMode(encoderRightA, INPUT_PULLUP);
  pinMode(encoderRightB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoderLeftA), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderRightA), rightEncoderISR, RISING);

  Serial.println("ESP32-S3 Ready");
}

// Loop
void loop(){
  //fast uart receive
  static char buffer[20];
  static int index = 0;

  while(Serial1.available()){
    char c = Serial1.read();
    if( c == '\n'){
      buffer[index] = '\0';
      index = 0;
 
      char* comma = strchr(buffer, ',');
      if(comma){
        *comma = '\0';
        ly = atoi (buffer); // atoi = ASCII to Integer
        rx = atoi (comma +1);

        Serial.print("LY: "); Serial.print(ly);
        Serial.print(" RX: "); Serial.print(rx);
      }
    } else if (index < sizeof(buffer) - 1){
      buffer[index++] = c;
    }
  }
  // Deadband for joystick drift
  if (abs(ly) < 10) ly = 0;
  if (abs(rx) < 10) rx = 0;

  compute2Wheel(ly,rx);
  applySpeed();

  // Odometry
  long deltaLeft = leftPulses - prevLeftPulses;
  long deltaRight = rightPulses - prevRightPulses;
  prevLeftPulses = leftPulses;
  prevRightPulses = rightPulses;

  float dLeft = (deltaLeft / pulsesPerRevLeft) * wheelCircumference;
  float dRight = (deltaRight / pulsesPerRevRight) * wheelCircumference;
  float dCenter = (dLeft + dRight) / 2.0;

  theta = readYaw();
  float thetaRad = theta * 3.14159 / 180.0;

  x += dCenter * cos(thetaRad);
  y += dCenter * sin(thetaRad);

// DEBUG PRINT (ALWAYS PRINTS)
Serial.print("  X=");
Serial.print(x, 2);
Serial.print(" Y=");
Serial.print(y, 2);
Serial.print(" Theta=");
Serial.println(theta, 2);

  if (digitalRead(PROX_PIN) == HIGH && !boxDetected){
    boxDetected = true;

    float boxX = x + BOX_OFFSET * cos(thetaRad);
    float boxY = y + BOX_OFFSET * sin(thetaRad);

    Serial.print("Box Detected at : ");
    Serial.print(" X: "); Serial.print(boxX, 3);
    Serial.print(" mm\tY: "); Serial.print(boxY, 3);
    Serial.print(" mm\tTheta: "); Serial.println(theta, 2);
  }

  if(digitalRead(PROX_PIN) == LOW){
    boxDetected = false;
  }

  delay(100);
}

/*
void setup() {
  Serial.begin(9600);                    // USB Serial (PC)
  Serial1.begin(9600, SERIAL_8N1, 47, 48); // RX=47, TX=48 (from ESP32)

  Serial.println("ESP32-S3 READY");
}

void loop() { 
  static char buffer[20];
  static int index = 0;

  while (Serial1.available()) {
    char c = Serial1.read();

    if (c == '\n') {
      buffer[index] = '\0';
      index = 0;

      int ly, rx;
      if (sscanf(buffer, "%d,%d", &ly, &rx) == 2) {
        Serial.print("LY: ");
        Serial.print(ly);
        Serial.print("  RX: ");
        Serial.println(rx);
      }
    }
    else if (index < sizeof(buffer) - 1) {
      buffer[index++] = c;
    }
  }
}
*/




