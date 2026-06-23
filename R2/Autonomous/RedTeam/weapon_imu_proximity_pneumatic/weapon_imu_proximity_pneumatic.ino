#include <Arduino.h>
#include <Servo.h>
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
const int wfr_dir_pin = 27;
const int wfl_dir_pin = 29;
const int wrr_dir_pin = 25;
const int wrl_dir_pin = 23;

const int wfr_pwm_pin = 6;
const int wfl_pwm_pin = 7;
const int wrr_pwm_pin = 5;
const int wrl_pwm_pin = 4;

// =====================================
// PROXIMITY SENSOR
// =====================================
const int proxPin = 12;

// =====================================
// SERVO — rotate 
// =====================================
Servo rotateServo;
const int rotateServoPin = 9;

// =====================================
// PNEUMATIC GRIPPER
// =====================================
const int pneumaticPin = 10;

unsigned long releaseStartTime = 0;
bool timerStarted = false;

// =====================================
// ENCODERS
// RR  — pin 18 
// FL  — pin 19 
// FR  — pin 2  
// RL  — pin 3  
// =====================================
const int outputA  = 18;  // RR interrupt
const int outputB  = 48;  // RR direction

const int outputA1 = 19;  // FL interrupt
const int outputB1 = 31;  // FL direction

const int outputA2 = 2;   // FR interrupt
const int outputB2 = 30;  // FR direction

const int outputA3 = 3;   // RL interrupt
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
long leftCounts1        = 500;
long leftCounts2        = 200;
long blindForwardCounts = 490;   
long forwardCounts     = 1500;
long diagonalCounts    = 650;
long turn90Counts      = 550;

// =====================================
// IMU / HEADING
// =====================================
float currentYaw         = 0.0;
float desiredHeading     = 0.0;
bool  headingInitialized = false;

// P gain 
const float kP_heading   = 7.5;
const int   maxCorrection = 30;

// =====================================
// STATE MACHINE
// state 0 — strafe left
// state 1 — NEW: move forward blind (proximity ignored) for blindForwardCounts
// state 2 — move forward + check proximity
// state 3 — move backward + check proximity
// state 4 — pneumatic ON (grip)
// state 5 — rotate spearhead servo
// state 6 — strafe right
// state 7 — rotate 90
// state 8 — wait then pneumatic OFF (release)
// state 9 — stop (finished)
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

  pinMode(wfr_dir_pin, OUTPUT);
  pinMode(wfl_dir_pin, OUTPUT);
  pinMode(wrr_dir_pin, OUTPUT);
  pinMode(wrl_dir_pin, OUTPUT);

  pinMode(wfr_pwm_pin, OUTPUT);
  pinMode(wfl_pwm_pin, OUTPUT);
  pinMode(wrr_pwm_pin, OUTPUT);
  pinMode(wrl_pwm_pin, OUTPUT);

  rotateServo.attach(rotateServoPin);
  rotateServo.write(90);

  pinMode(pneumaticPin, OUTPUT);
  digitalWrite(pneumaticPin, LOW);   // initially OFF

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
  // STATE 0 — STRAFE LEFT
  // Heading locked at 0°
  // =====================================
  if (state == 0)
  {
    if (avgCounts < leftCounts1)
    {
      strafeLeft(50);
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
  // STATE 1 — BLIND FORWARD POSE
  // Move forward for blindForwardCounts.
  // Proximity sensor is ignored here.
  // =====================================
  else if (state == 1)
  {
    if (avgCounts < blindForwardCounts)
    {
      moveForward(40);
    }
    else
    {
      stopRobot();
      resetEncoders();
      delay(500);
      state = 2;
    }
  }

  // =====================================
  // STATE 2 — MOVE FORWARD + CHECK PROX
  // Heading locked at 0°
  // If prox LOW  → State 4 (pneumatic grip)
  // If steps done → State 3 (move backward scan)
  // =====================================
  else if (state == 2)
  {
    if (digitalRead(proxPin) == LOW)
    {
      stopRobot();
      Serial.println("SPEARHEAD FOUND (forward pass)");
      resetEncoders();
      state = 3.5;
    }
    else if (avgCounts < forwardCounts)
    {
      moveForward(40);
    }
    else
    {
      stopRobot();
      resetEncoders();
      delay(500);
      state = 3;
    }
  }

  // =====================================
  // STATE 3 — MOVE BACKWARD + CHECK PROX
  // Heading locked at 0°
  // If prox LOW  → State 4 (pneumatic grip)
  // If steps done → State 9 (not found, stop)
  // =====================================
  else if (state == 3)
  {
    if (digitalRead(proxPin) == LOW)
    {
      stopRobot();
      Serial.println("SPEARHEAD FOUND (backward pass)");
      resetEncoders();
      state = 3.5;
    }
    else if (avgCounts < forwardCounts)
    {
      moveBackward(40);
    }
    else
    {
      stopRobot();
      Serial.println("SPEARHEAD NOT FOUND");
      state = 9;
    }
  }

// STATE 3.5
  else if (state == 3.5){
    if (avgCounts < leftCounts2)
    {
      strafeLeft(50);
    }
    else
    {
      stopRobot();
      resetEncoders();
      delay(500);
      state = 4;
    }

  }

  // =====================================
  // STATE 4 — PNEUMATIC GRIP ON
  // =====================================
  else if (state == 4)
  {
    stopRobot();
    delay(1000);

    digitalWrite(pneumaticPin, HIGH);
    Serial.println("PNEUMATIC ON");
    delay(500);   // adjust as needed

    state = 5;
  }

  // =====================================
  // STATE 5 — ROTATE SPEARHEAD SERVO
  // =====================================
  else if (state == 5)
  {
    rotateServo.write(0);
    Serial.println("ROTATE SERVO TO 70");
    delay(1000);
    resetEncoders();
    state = 6;
  }

  // =====================================
  // STATE 6 — STRAFE RIGHT
  // Heading locked at 0°
  // =====================================
  else if (state == 6)
  {
    long strafeCounts =
      (abs(counter) + abs(counter1) + abs(counter2) + abs(counter3)) / 4;

    if (strafeCounts < diagonalCounts)
    {
      strafeRight(50);
    }
    else
    {
      stopRobot();
      resetEncoders();
      delay(500);
      Serial.println("STRAFE RIGHT COMPLETE");
      state = 7;
    }
  }

  // =====================================
  // STATE 7 — ROTATE 90°
  // Heading lock DISABLED — pure rotation
  // =====================================
  else if (state == 7)
  {
    if (avgCounts < turn90Counts)
    {
      rotateRight(50);
    }
    else
    {
      stopRobot();
      releaseStartTime = millis();
      timerStarted     = true;
      Serial.println("TURN 90 COMPLETE");
      state = 8;
    }
  }

  // =====================================
  // STATE 8 — WAIT 30s THEN PNEUMATIC OFF
  // =====================================
  else if (state == 8)
  {
    if (timerStarted && millis() - releaseStartTime >= 30000)
    {
      digitalWrite(pneumaticPin, LOW);
      Serial.println("PNEUMATIC OFF");

      state = 9;
    }
  }

  // =====================================
  // STATE 9 — STOP (FINISHED)
  // =====================================
  else if (state == 9)
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
// STRAFE RIGHT — P heading lock
// Front pair vs rear pair correction
// Positive err (drifted CW) → reduce front, boost rear
// =====================================
void strafeRight(int pwm)
{
  int corr     = getCorrection();
  int pwmRear  = constrain(pwm - corr, 0, 255);
  int pwmFront = constrain(pwm + corr, 0, 255);

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
