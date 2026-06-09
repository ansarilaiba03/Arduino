#include <Arduino.h>
#include <Servo.h>

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
const int proxPin = 33;

// =====================================
// SERVOS
// =====================================
Servo gripperServo;
Servo rotateServo;

const int gripperServoPin = 2;
const int rotateServoPin  = 3;

unsigned long releaseStartTime = 0;
bool timerStarted = false;

// =====================================
// ENCODERS
// =====================================
int outputA  = 18;  // RR
int outputB  = 48;

int outputA1 = 19;  // FL
int outputB1 = 38;

int outputA2 = 20;  // FR
int outputB2 = 30;

int outputA3 = 21;  // RL
int outputB3 = 22;

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
const float kP_heading    = 0.8;
const int   maxCorrection = 30;

// =====================================
// STATE MACHINE
// =====================================
int state = 0;

// =====================================
// FUNCTION DECLARATIONS
// =====================================
void readIMU();
float headingError();
int getCorrection();
void strafeLeft(int pwm);
void moveBackward(int pwm);
void moveForward(int pwm);
void rotateRight(int pwm);
void stopRobot();
void resetEncoders();
void readEncoderA();
void readEncoderA1();
void readEncoderA2();
void readEncoderA3();

// =====================================
// IMU READ — Serial3 (RX3=15, TX3=14)
// =====================================
void readIMU()
{
  while (Serial3.available())
  {
    String line = Serial3.readStringUntil('\n');
    line.trim();
    if (line.startsWith("Y:"))
      currentYaw = line.substring(2).toFloat();
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
  Serial3.begin(115200);

  pinMode(wfr_dir_pin, OUTPUT);
  pinMode(wfl_dir_pin, OUTPUT);
  pinMode(wrr_dir_pin, OUTPUT);
  pinMode(wrl_dir_pin, OUTPUT);

  pinMode(wfr_pwm_pin, OUTPUT);
  pinMode(wfl_pwm_pin, OUTPUT);
  pinMode(wrr_pwm_pin, OUTPUT);
  pinMode(wrl_pwm_pin, OUTPUT);

  gripperServo.attach(gripperServoPin);
  rotateServo.attach(rotateServoPin);

  gripperServo.write(90);
  rotateServo.write(0);

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
  // STATE 1 — MOVE BACKWARD + CHECK PROX
  // Heading locked at 0°
  // If prox LOW  → State 3 (close gripper)
  // If steps done → State 2 (forward scan)
  // =====================================
  else if (state == 1)
  {
    if (digitalRead(proxPin) == LOW)
    {
      stopRobot();
      Serial.println("SPEARHEAD FOUND (backward pass)");
      resetEncoders();
      state = 3;
    }
    else if (avgCounts < forwardCounts)
    {
      moveBackward(60);
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
  // Reverse scan back to start position
  // If prox LOW  → State 3 (close gripper)
  // If steps done → State 8 (not found, stop)
  // =====================================
  else if (state == 2)
  {
    if (digitalRead(proxPin) == LOW)
    {
      stopRobot();
      Serial.println("SPEARHEAD FOUND (forward pass)");
      resetEncoders();
      state = 3;
    }
    else if (avgCounts < forwardCounts)
    {
      moveForward(60);
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
    delay(2000);
    gripperServo.write(0);
    Serial.println("GRIPPER CLOSED");
    delay(1000);
    state = 4;
  }

  // =====================================
  // STATE 4 — ROTATE SPEARHEAD SERVO
  // =====================================
  else if (state == 4)
  {
    rotateServo.write(90);
    Serial.println("ROTATE SERVO TO 90");
    delay(1000);
    resetEncoders();
    state = 5;
  }

  // =====================================
  // STATE 5 — STRAFE LEFT
  // Heading locked at 0°
  // =====================================
  else if (state == 5)
  {
    long strafeCounts =
      (abs(counter) + abs(counter1) + abs(counter2) + abs(counter3)) / 4;

    if (strafeCounts < diagonalCounts)
    {
      strafeLeft(80);
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
// Correction: left side vs right side
// Positive err (drifted CW) → boost left, reduce right
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
// Correction: left side vs right side
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
// For mecanum strafe, yaw correction is
// applied as front pair vs rear pair:
// Positive err (drifted CW) → boost rear, reduce front
// =====================================
void strafeLeft(int pwm)
{
  int corr     = getCorrection();
  int pwmFront = constrain(pwm - corr, 0, 255);
  int pwmRear  = constrain(pwm + corr, 0, 255);

  // Strafe left directions (unchanged from your original)
  digitalWrite(wfl_dir_pin, LOW);
  digitalWrite(wrr_dir_pin, LOW);
  digitalWrite(wfr_dir_pin, LOW);
  digitalWrite(wrl_dir_pin, HIGH);

  // Front pair gets pwmFront, rear pair gets pwmRear
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