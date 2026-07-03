#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// =====================================
// BNO055 IMU — I2C on Mega pins 20(SDA) 21(SCL)
// =====================================
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// =====================================
// MOTOR PINS
// =====================================

const int wfr_dir_pin = 27;  //orange
const int wfl_dir_pin = 29; //red
const int wrr_dir_pin = 25; //yellow
const int wrl_dir_pin = 22; //green

const int wfr_pwm_pin = 6; //orange
const int wfl_pwm_pin = 7; //black
const int wrr_pwm_pin = 11; 
const int wrl_pwm_pin = 4; 

// =====================================
// PROXIMITY SENSOR
// =====================================
const int proxPin = 12; //purple

// =====================================
// STEPPER MOTOR 
// =====================================
#define STEP_PIN 17 
#define DIR_PIN  16

unsigned long releaseStartTime = 0;
bool timerStarted = false;

// =====================================
// PNEUMATIC GRIPPER
// =====================================
const int pneumaticPin = 10; // green

// =====================================
// ENCODERS
// RR  — pin 18
// FL  — pin 19
// FR  — pin 2
// RL  — pin 3
// =====================================
const int outputA  = 18;
const int outputB  = 41;

const int outputA1 = 19;
const int outputB1 = 45;

const int outputA2 = 2;
const int outputB2 = 49;

const int outputA3 = 3;
const int outputB3 = 53;

// =====================================
// COUNTERS
// =====================================
volatile long counter  = 0;   // RR
volatile long counter1 = 0;   // FL
volatile long counter2 = 0;   // FR
volatile long counter3 = 0;   // RL


void readEncoderA();
void readEncoderA1();
void readEncoderA2();
void readEncoderA3();

// =====================================
// DISTANCES (TUNE THESE)
// =====================================
long forwardCounts  = 100;   // state 0 target
long turn90Counts   = 600;   // state 1 target

// =====================================
// IMU / HEADING
// =====================================
float currentYaw         = 0.0;
float desiredHeading     = 0.0;
bool  headingInitialized = false;

const float kP_heading    = 12.0;
const int   maxCorrection = 30;

// =====================================
// STATE MACHINE
//   state 0 — move forward, stop at 100 counts
//   state 1 — rotate 90° right
//   state 2 — stop (finished)
//
//   Pneumatic is set HIGH once in setup()
//   and stays HIGH throughout states 0-2.
// =====================================
int state = 0;

// =====================================
// FUNCTION DECLARATIONS
// =====================================
void readIMU();
float headingError();
int   getCorrection();
void  moveForward(int pwm);
void  rotateRight(int pwm);
void  stopRobot();
void  resetEncoders();

// =====================================
// READ IMU
// =====================================
void readIMU()
{
  imu::Quaternion quat = bno.getQuat();
  double w = quat.w();
  double x = quat.x();
  double y = quat.y();
  double z = quat.z();

  double sinYaw = 2.0 * (w * z + x * y);
  double cosYaw = 1.0 - 2.0 * (y * y + z * z);
  currentYaw    = (float)(atan2(sinYaw, cosYaw) * 180.0 / PI);
}

// =====================================
// HEADING ERROR (handles wraparound)
// =====================================
float headingError()
{
  float err = currentYaw - desiredHeading;
  if (err >  180.0) err -= 360.0;
  if (err < -180.0) err += 360.0;
  return err;
}

// =====================================
// P CORRECTION
// =====================================
int getCorrection()
{
  int corr = (int)(kP_heading * headingError());
  return constrain(corr, -maxCorrection, maxCorrection);
}

// =====================================
// SETUP
// =====================================
void setup()
{
  Serial.begin(115200);

  if (!bno.begin())
  {
    Serial.println("BNO055 NOT FOUND — check wiring!");
    while (1);
  }
  bno.setExtCrystalUse(true);
  delay(1000);
  Serial.println("BNO055 ready");

  pinMode(wfr_dir_pin, OUTPUT);
  pinMode(wfl_dir_pin, OUTPUT);
  pinMode(wrr_dir_pin, OUTPUT);
  pinMode(wrl_dir_pin, OUTPUT);

  pinMode(wfr_pwm_pin, OUTPUT);
  pinMode(wfl_pwm_pin, OUTPUT);
  pinMode(wrr_pwm_pin, OUTPUT);
  pinMode(wrl_pwm_pin, OUTPUT);

  // Stepper
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN,  OUTPUT);

  // Pneumatic — turn ON now, stays HIGH throughout states 0-2
  pinMode(pneumaticPin, OUTPUT);
  digitalWrite(pneumaticPin, HIGH);
  Serial.println("PNEUMATIC ON (held HIGH throughout)");

  // Proximity
  pinMode(proxPin, INPUT);

  pinMode(outputA,  INPUT_PULLUP);
  pinMode(outputB,  INPUT_PULLUP);
  pinMode(outputA1, INPUT_PULLUP);
  pinMode(outputB1, INPUT_PULLUP);
  pinMode(outputA2, INPUT_PULLUP);
  pinMode(outputB2, INPUT_PULLUP);
  pinMode(outputA3, INPUT_PULLUP);
  pinMode(outputB3, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(outputA),  readEncoderA,  RISING);
  attachInterrupt(digitalPinToInterrupt(outputA1), readEncoderA1, RISING);
  attachInterrupt(digitalPinToInterrupt(outputA2), readEncoderA2, RISING);
  attachInterrupt(digitalPinToInterrupt(outputA3), readEncoderA3, RISING);

  delay(2000);
  resetEncoders();
}

// =====================================
// LOOP
// =====================================
void loop()
{
  readIMU();

  if (!headingInitialized)
  {
    desiredHeading     = currentYaw;
    headingInitialized = true;
    Serial.print("Reference Heading = ");
    Serial.println(desiredHeading);
  }

  long avgCounts =
    (abs(counter) + abs(counter1) + abs(counter2) + abs(counter3)) / 4;

  Serial.print("FL=");     Serial.print(counter1);
  Serial.print(" FR=");    Serial.print(counter2);
  Serial.print(" RR=");    Serial.print(counter);
  Serial.print(" RL=");    Serial.print(counter3);
  Serial.print(" AVG=");   Serial.print(avgCounts);
  Serial.print(" YAW=");   Serial.print(currentYaw);
  Serial.print(" ERR=");   Serial.print(headingError());
  Serial.print(" STATE="); Serial.println(state);

  // =====================================
  // STATE 0 — MOVE FORWARD, STOP AT 100 COUNTS
  // =====================================
  if (state == 0)
  {
    if (avgCounts < forwardCounts)
      moveBackward(40);
    else
    {
      stopRobot();
      resetEncoders();
      delay(500);
      Serial.println("STATE 0: forward complete");
      state = 1;
    }
  }

  // =====================================
  // STATE 1 — ROTATE 90° RIGHT
  // =====================================
  else if (state == 1)
  {
    if (avgCounts < turn90Counts)
      rotateRight(50);
    else
    {
      stopRobot();
      resetEncoders();
      releaseStartTime = millis();
      timerStarted     = true;
      //holdInitialized  = false; 
      delay(500);
      Serial.println("STATE 1: rotate 90 complete");
      state = 2;
    }
  }
  // =====================================
  // STATE 2 - wait 40sec
  // =====================================

   else if (state == 2)
  {
    if (timerStarted && millis() - releaseStartTime >= 40000)
    {
      stopRobot();
      digitalWrite(pneumaticPin, LOW);
      timerStarted = false;
      Serial.println("PNEUMATIC OFF");
      state = 3;
    }
}

  // =====================================
  // STATE 3 — STOP (FINISHED)
  // =====================================
  else if (state == 3)
  {
    stopRobot();
  }

  delay(10);
}


// =====================================
// MOVE FORWARD — P heading lock
// =====================================
void moveForward(int pwm)
{
  int corr     = getCorrection();
  int pwmLeft  = constrain(pwm + corr, 0, 255);
  int pwmRight = constrain(pwm - corr, 0, 255);

  digitalWrite(wfr_dir_pin, HIGH);
  digitalWrite(wrl_dir_pin, LOW);
  digitalWrite(wfl_dir_pin, LOW);
  digitalWrite(wrr_dir_pin, LOW);

  analogWrite(wfr_pwm_pin, pwmRight);
  analogWrite(wrr_pwm_pin, pwmRight);
  analogWrite(wfl_pwm_pin, pwmLeft);
  analogWrite(wrl_pwm_pin, pwmLeft);
}

// =====================================
// ROTATE RIGHT — NO heading lock
// =====================================
void rotateRight(int pwm)
{
  digitalWrite(wfr_dir_pin, LOW);
  digitalWrite(wrr_dir_pin, HIGH);
  digitalWrite(wfl_dir_pin, LOW);
  digitalWrite(wrl_dir_pin, LOW);

  analogWrite(wfr_pwm_pin, pwm);
  analogWrite(wrr_pwm_pin, pwm);
  analogWrite(wfl_pwm_pin, pwm);
  analogWrite(wrl_pwm_pin, pwm);
}

// =====================================
// STOP
// =====================================
void stopRobot()
{
  analogWrite(wfr_pwm_pin, 0);
  analogWrite(wfl_pwm_pin, 0);
  analogWrite(wrr_pwm_pin, 0);
  analogWrite(wrl_pwm_pin, 0);
}

// =====================================
// RESET ENCODERS
// =====================================
void resetEncoders()
{
  counter  = 0;
  counter1 = 0;
  counter2 = 0;
  counter3 = 0;
}

// =====================================
// ENCODER ISRs
// =====================================
void readEncoderA()
{
  if (digitalRead(outputA) == digitalRead(outputB)) counter++;
  else counter--;
}

void readEncoderA1()
{
  if (digitalRead(outputA1) == digitalRead(outputB1)) counter1++;
  else counter1--;
}

void readEncoderA2()
{
  if (digitalRead(outputA2) == digitalRead(outputB2)) counter2++;
  else counter2--;
}

void readEncoderA3()
{
  if (digitalRead(outputA3) == digitalRead(outputB3)) counter3--;
  else counter3++;
}
