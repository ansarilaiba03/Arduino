#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "HUSKYLENS.h"
#include "SoftwareSerial.h"

// =====================================
// BNO055 IMU — I2C on Mega pins 20(SDA) 21(SCL)
// =====================================
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// =====================================
// HUSKYLENS — SoftwareSerial on pins 12(RX), 13(TX)
// Mega pin 12 -> HuskyLens TX
// Mega pin 13 -> HuskyLens RX
// (Swap the two pin numbers below if your wiring is reversed)
// =====================================
SoftwareSerial mySerial(12, 13); // RX, TX
HUSKYLENS huskylens;
bool objectDetected = false;   // debounced/filtered result — use this everywhere else

// ----- Persistence filter (debounce) -----
// Requires this many CONSECUTIVE frames in agreement
// before objectDetected actually flips, so a single
// noisy/dropped HuskyLens frame can't cause a flicker.
const int DETECTION_FILTER_COUNT = 10; // tune: higher = steadier but slower to react
int detectionStreak   = 0;
int noDetectionStreak = 0;

// =====================================
// MOTOR PINS
// =====================================
const int wfr_dir_pin = 27;
const int wfl_dir_pin = 29;
const int wrr_dir_pin = 25;
const int wrl_dir_pin = 23;

const int wfr_pwm_pin = 6;
const int wfl_pwm_pin = 7;
const int wrr_pwm_pin = 5;
const int wrl_pwm_pin = 4;

// =====================================
// SERVOS — pins 8 and 9
// =====================================
Servo gripperServo;
Servo rotateServo;

const int gripperServoPin = 10;
const int rotateServoPin  = 9;

unsigned long releaseStartTime = 0;
bool timerStarted = false;

// const int gripDirPin = 40;
// const int gripPwmPin = 10;

// =====================================
// ENCODERS
// RR  — pin 18 (interrupt)
// FL  — pin 19 (interrupt)
// FR  — pin 2  (interrupt, moved from 20 — 20 now used by I2C SDA)
// RL  — pin 3  (interrupt, moved from 21 — 21 now used by I2C SCL)
// =====================================
const int outputA  = 18;  // RR interrupt
const int outputB  = 48;  // RR direction

const int outputA1 = 19;  // FL interrupt
const int outputB1 = 31;  // FL direction

const int outputA2 = 2;   // FR interrupt (moved from 20)  
const int outputB2 = 30;  // FR direction

const int outputA3 = 3;   // RL interrupt (moved from 21)
const int outputB3 = 22;  // RL direction

// =====================================
// COUNTERS
// =====================================
volatile long counter  = 0;   // RR
volatile long counter1 = 0;   // FL
volatile long counter2 = 0;   // FR
volatile long counter3 = 0;   // RL

// =====================================
// DISTANCES (TUNE THESE)
// =====================================
long leftCounts     = 730;
long forwardCounts  = 1500; 
long diagonalCounts = 700;
long turn90Counts   = 600;

// =====================================
// IMU / HEADING
// =====================================
float currentYaw         = 0.0;
float desiredHeading     = 0.0;
bool  headingInitialized = false;

// P gain — tune this (start at 0.8)
const float kP_heading   = 5.5;
const int   maxCorrection = 30;

// =====================================
// STATE MACHINE
// =====================================
int state = 0;

// =====================================
// READ IMU — quaternion yaw from BNO055
// =====================================
void readIMU()
{
  imu::Quaternion quat = bno.getQuat();

  double w = quat.w();
  double x = quat.x();
  double y = quat.y();
  double z = quat.z();

  // Yaw from quaternion (ZYX convention)
  double sinYaw = 2.0 * (w * z + x * y);
  double cosYaw = 1.0 - 2.0 * (y * y + z * z);
  currentYaw    = (float)(atan2(sinYaw, cosYaw) * 180.0 / PI);
}

// =====================================
// READ HUSKYLENS — updates objectDetected
// Raw per-frame result is passed through a
// persistence filter (see DETECTION_FILTER_COUNT)
// before objectDetected is allowed to change,
// so single noisy/dropped frames don't flicker it
// =====================================
void readHuskylens()
{
  bool rawDetected = false;

  if (huskylens.request())
  {
    if (huskylens.isLearned() && huskylens.available())
    {
      rawDetected = true;

      while (huskylens.available())
      {
        HUSKYLENSResult result = huskylens.read();
        if (result.command == COMMAND_RETURN_BLOCK)
        {
          Serial.print(F("HuskyLens Block: ID="));
          Serial.print(result.ID);
          Serial.print(F(" xCenter="));
          Serial.print(result.xCenter);
          Serial.print(F(" yCenter="));
          Serial.println(result.yCenter);
        }
        else if (result.command == COMMAND_RETURN_ARROW)
        {
          Serial.print(F("HuskyLens Arrow: ID="));
          Serial.println(result.ID);
        }
      }
    }
  }

  // ----- Persistence filter (debounce) -----
  if (rawDetected)
  {
    detectionStreak++;
    noDetectionStreak = 0;
    if (detectionStreak >= DETECTION_FILTER_COUNT)
    {
      objectDetected = true;
    }
  }
  else
  {
    noDetectionStreak++;
    detectionStreak = 0;
    if (noDetectionStreak >= DETECTION_FILTER_COUNT)
    {
      objectDetected = false;
    }
  }
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
// Positive err = drifted clockwise (right)
// =====================================
int getCorrection()
{
  int corr = (int)(kP_heading * headingError());
  corr = constrain(corr, -maxCorrection, maxCorrection);
  return corr;
}

// =====================================
// SETUP
// =====================================
void setup()
{
  Serial.begin(115200);

  // Init BNO055
  if (!bno.begin())
  {
    Serial.println("BNO055 NOT FOUND — check wiring!");
    while (1);
  }
  bno.setExtCrystalUse(true);
  delay(1000);
  Serial.println("BNO055 ready");

  // Init HuskyLens over SoftwareSerial (pins 12=RX, 13=TX)
  mySerial.begin(9600);
  while (!huskylens.begin(mySerial))
  {
    Serial.println(F("HuskyLens begin failed!"));
    Serial.println(F("1.Recheck the \"Protocol Type\" in HuskyLens (General Settings>>Protocol Type>>Serial 9600)"));
    Serial.println(F("2.Recheck the wiring (HuskyLens TX -> Mega pin 12, HuskyLens RX -> Mega pin 13)."));
    delay(100);
  }
  Serial.println("HuskyLens ready");

  pinMode(wfr_dir_pin, OUTPUT);
  pinMode(wfl_dir_pin, OUTPUT);
  pinMode(wrr_dir_pin, OUTPUT);
  pinMode(wrl_dir_pin, OUTPUT);

  pinMode(wfr_pwm_pin, OUTPUT);
  pinMode(wfl_pwm_pin, OUTPUT);
  pinMode(wrr_pwm_pin, OUTPUT);
  pinMode(wrl_pwm_pin, OUTPUT);

  rotateServo.attach(rotateServoPin);
  rotateServo.write(0);

  gripperServo.attach(gripperServoPin);
  gripperServo.write(90);
  
  // pinMode(gripDirPin, OUTPUT);
  // pinMode(gripPwmPin, OUTPUT);
  // analogWrite(gripPwmPin, 0);

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
  readHuskylens();

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
  // HOLD SERVOS STEADY UNTIL AN OBJECT
  // IS ACTUALLY FOUND (states 0,1,2)
  // Re-writing every loop instead of just
  // once in setup() stops them drifting/
  // flickering from noise (e.g. SoftwareSerial)
  // =====================================
  if (state < 3 && !objectDetected)
  {
    gripperServo.write(90);
    rotateServo.write(0);
  }

  // =====================================
  // STATE 0 — STRAFE LEFT
  // Heading locked at 0°
  // =====================================
  if (state == 0)
  {
    if (avgCounts < leftCounts)
    {
      strafeLeft(80);
    }
    else
    {
      stopRobot();
      resetEncoders();
      delay(500);
      state = 1;
    }
  }

  // =====================================
  // STATE 1 — MOVE FORWARD + CHECK HUSKYLENS
  // Heading locked at 0°
  // If object detected → State 3 (close gripper)
  // If steps done → State 2 (forward scan)
  // =====================================
  else if (state == 1)
  {
    if (objectDetected)
    {
      stopRobot();
      Serial.println("SPEARHEAD FOUND (backward pass)");
      resetEncoders();
      state = 3;
    }
    else if (avgCounts < forwardCounts)
    {
      moveForward(50);
    }
    else
    {
      stopRobot();
      resetEncoders();
      delay(500);
      state = 8;
    }
  }

  // =====================================
  // STATE 2 — MOVE BACKWARD + CHECK HUSKYLENS
  // Heading locked at 0°
  // If object detected → State 3 (close gripper)
  // If steps done → State 8 (not found)
  // =====================================
  else if (state == 2)
  {
    if (objectDetected)
    {
      stopRobot();
      Serial.println("SPEARHEAD FOUND (forward pass)");
      resetEncoders();
      state = 3;
    }
    else if (avgCounts < forwardCounts)
    {
      moveBackward(40);
    }
    else
    {
      stopRobot();
      Serial.println("SPEARHEAD NOT FOUND");
      state = 8;
    }
  }

  // =====================================
  // STATE 3 — CLOSE GRIPPER
  // =====================================
  else if (state == 3)
  {
    stopRobot();
    delay(1000);
    gripperServo.write(0);
    Serial.println("GRIPPER CLOSED");
    delay(1000);

    // closeGripper();
    // delay(400);   // tune experimentally
    // stopGripper();

    state = 4;
  }

  // =====================================
  // STATE 4 — ROTATE SPEARHEAD SERVO
  // =====================================
  else if (state == 4)
  {
    rotateServo.write(70);
    Serial.println("ROTATE SERVO TO 90");
    delay(1000);
    resetEncoders();
    state = 5;
  }

  // =====================================
  // STATE 5 — STRAFE Right
  // Heading locked at 0°
  // =====================================
  else if (state == 5)
  {
    long strafeCounts =
      (abs(counter) + abs(counter1) + abs(counter2) + abs(counter3)) / 4;

    if (strafeCounts < diagonalCounts)
    {
      strafeRight(80);
    }
    else
    {
      stopRobot();
      resetEncoders();
      delay(500);
      Serial.println("STRAFE LEFT COMPLETE");
      state = 6;
    }
  }

  // =====================================
  // STATE 6 — ROTATE 90°
  // Heading lock DISABLED — pure rotation
  // =====================================
  else if (state == 6)
  {
    if (avgCounts < turn90Counts)
    {
      rotateRight(80);
    }
    else
    {
      stopRobot();
      releaseStartTime = millis();
      timerStarted     = true;
      Serial.println("TURN 90 COMPLETE");
      state = 7;
    }
  }

  // =====================================
  // STATE 7 — WAIT 30s THEN OPEN GRIPPER
  // =====================================
  else if (state == 7)
  {
    if (timerStarted && millis() - releaseStartTime >= 30000)
    {
      gripperServo.write(100);
      Serial.println("GRIPPER OPENED");

      // openGripper();
      // delay(400);   // tune experimentally
      // stopGripper();
      // Serial.println("GRIPPER OPENED");

      state = 8;
    }
  }

  // =====================================
  // STATE 8 — STOP (FINISHED)
  // =====================================
  else if (state == 8)
  {
    stopRobot();
  }

  delay(10);
}

// =====================================
// MOVE BACKWARD — P heading lock
// Left side vs right side correction
// =====================================
void moveBackward(int pwm)
{
  int corr     = getCorrection();
  int pwmLeft  = constrain(pwm + corr, 0, 255);
  int pwmRight = constrain(pwm - corr, 0, 255);

  digitalWrite(wfr_dir_pin, LOW);
  digitalWrite(wrl_dir_pin, HIGH);
  digitalWrite(wfl_dir_pin, HIGH);
  digitalWrite(wrr_dir_pin, HIGH);

  analogWrite(wfr_pwm_pin, pwmRight);
  analogWrite(wrr_pwm_pin, pwmRight);
  analogWrite(wfl_pwm_pin, pwmLeft);
  analogWrite(wrl_pwm_pin, pwmLeft);
}

// =====================================
// MOVE FORWARD — P heading lock
// Left side vs right side correction
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
// STRAFE LEFT — P heading lock
// Front pair vs rear pair correction
// Positive err (drifted CW) → reduce front, boost rear
// =====================================
void strafeLeft(int pwm)
{
  int corr     = getCorrection();
  int pwmFront = constrain(pwm - corr, 0, 255);
  int pwmRear  = constrain(pwm + corr, 0, 255);

  digitalWrite(wfl_dir_pin, HIGH);
  digitalWrite(wrr_dir_pin, HIGH);
  digitalWrite(wfr_dir_pin, HIGH);
  digitalWrite(wrl_dir_pin, LOW);

  analogWrite(wfl_pwm_pin, pwmFront);
  analogWrite(wfr_pwm_pin, pwmFront);
  analogWrite(wrr_pwm_pin, pwmRear);
  analogWrite(wrl_pwm_pin, pwmRear);
}

// =====================================
// STRAFE Right — P heading lock
// Front pair vs rear pair correction
// Positive err (drifted CW) → reduce front, boost rear
// =====================================
void strafeRight(int pwm)
{
  int corr     = getCorrection();
  int pwmRear = constrain(pwm - corr, 0, 255);
  int pwmFront  = constrain(pwm + corr, 0, 255);

  digitalWrite(wfl_dir_pin, LOW);
  digitalWrite(wrr_dir_pin, LOW);
  digitalWrite(wfr_dir_pin, LOW);
  digitalWrite(wrl_dir_pin, HIGH);

  analogWrite(wfl_pwm_pin, pwmFront);
  analogWrite(wfr_pwm_pin, pwmFront);
  analogWrite(wrr_pwm_pin, pwmRear);
  analogWrite(wrl_pwm_pin, pwmRear);
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

// void closeGripper()
// {
//   digitalWrite(gripDirPin, HIGH);
//   analogWrite(gripPwmPin, 150);
// }

// void openGripper()
// {
//   digitalWrite(gripDirPin, LOW);
//   analogWrite(gripPwmPin, 150);
// }

// void stopGripper()
// {
//   analogWrite(gripPwmPin, 0);
// }

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
