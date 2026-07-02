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
// const int wfr_dir_pin = 23;
// const int wfl_dir_pin = 25;
// const int wrr_dir_pin = 29;
// const int wrl_dir_pin = 27;

// const int wfr_pwm_pin = 7;
// const int wfl_pwm_pin = 6;
// const int wrr_pwm_pin = 4;
// const int wrl_pwm_pin = 5;

const int wfr_dir_pin = 27;  //orange
const int wfl_dir_pin = 29; //red
const int wrr_dir_pin = 25; //yellow
const int wrl_dir_pin = 23; //green

const int wfr_pwm_pin = 6; //orange
const int wfl_pwm_pin = 7; //black
const int wrr_pwm_pin = 5; 
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

unsigned long stepPrevMillis       = 0;
const unsigned long stepInterval   = 4;   // ms per half-step toggle
bool   stepPinState                = false;
int    stepCount                   = 0;
const int STEPS_90                 = 135;  // (90/360)*200 = 50 steps

// =====================================
// PNEUMATIC GRIPPER
// =====================================
const int pneumaticPin = 10; // green

unsigned long releaseStartTime = 0;
bool timerStarted    = false;

// =====================================
// POSITION HOLD (STATE 13)
// =====================================
bool holdInitialized = false;
long holdRR  = 0;
long holdFL  = 0;
long holdFR  = 0;
long holdRL  = 0;

const float kP_pos     = 0.8;   //encoder counts = 0 
const int   maxHoldPWM = 40;

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

// =====================================
// DISTANCES (TUNE THESE)
// =====================================
long leftCounts1         = 500;
long leftCounts2         = 200;
long blindForwardCounts  = 420;
long forwardCounts       = 1500;
long diagonalCounts      = 650;
long turn90Counts        = 600;

// =====================================
// IMU / HEADING
// =====================================
float currentYaw         = 0.0;
float desiredHeading     = 0.0;
bool  headingInitialized = false;

const float kP_heading   = 12.0;
const int   maxCorrection = 30;

// =====================================
// STATE MACHINE
//    — strafe left (leftCounts1)
//    — blind forward (blindForwardCounts, prox ignored)
//    — move forward + check prox
//    — move backward + check prox
//   — small strafe left (leftCounts2) after spearhead found
//    — pneumatic ON, re-check prox; lost → undo → state 2
//    — kick off stepper FORWARD (non-blocking) → state 7
//   — tick stepper forward; done → check prox → state 11 or state 8
//   — kick off stepper BACKWARD (undo) → state 9
//   — tick stepper backward; done → pneumatic OFF → state 2
//    — strafe right
//    — rotate 90°
//    — wait 30s with encoder position hold, then pneumatic OFF
//    — stop (finished)
// =====================================
int state = 0;

// =====================================
// FUNCTION DECLARATIONS
// =====================================
void readIMU();
float headingError();
int   getCorrection();
void  moveForward(int pwm);
void  moveBackward(int pwm);
void  strafeLeft(int pwm);
void  strafeRight(int pwm);
void  rotateRight(int pwm);
void  stopRobot();
void  resetEncoders();
void  holdPosition();
void  startStepper(bool forward);
bool  tickStepper();

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
// STEPPER — start motion
// forward=true  → DIR HIGH (rotate 90°)
// forward=false → DIR LOW  (reverse back)
// =====================================
void startStepper(bool forward)
{
  stepCount    = 0;
  stepPinState = false;
  digitalWrite(DIR_PIN,  forward ? HIGH : LOW);
  digitalWrite(STEP_PIN, LOW);
  stepPrevMillis = millis();
  Serial.println(forward ? "STEPPER: moving forward 90°"
                          : "STEPPER: reversing back 90°");
}

// =====================================
// STEPPER — non-blocking tick
// Call every loop iteration while stepper is active.
// Returns true when STEPS_90 steps are complete.
// =====================================
bool tickStepper()
{
  if (stepCount >= STEPS_90)
  {
    digitalWrite(STEP_PIN, LOW);   // leave pin LOW when done
    return true;
  }

  unsigned long now = millis();
  if (now - stepPrevMillis >= stepInterval)
  {
    stepPrevMillis = now;
    stepPinState   = !stepPinState;
    digitalWrite(STEP_PIN, stepPinState);

    if (stepPinState == HIGH)      // rising edge = one full step
      stepCount++;
  }
  return false;
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

  Serial.println("Holding at 0°...");
  // Send a slow trickle of forward pulses to resist gravity
  // Adjust holdInterval to minimum needed — too fast = overshoots
  const unsigned long holdInterval = 4; // ms between pulses, tune this
  unsigned long lastHoldStep = millis();

  // Hold until setup finishes (or you can just leave it running)
  // This is blocking — size the duration to your needs
  unsigned long holdDuration = 720; // hold for 3 seconds while setup continues
  unsigned long holdStart = millis();

  digitalWrite(DIR_PIN, HIGH); // HIGH = direction that fights gravity
  while (millis() - holdStart < holdDuration)
  {
    if (millis() - lastHoldStep >= holdInterval)
    {
      lastHoldStep = millis();
      digitalWrite(STEP_PIN, HIGH);
      delay(2);
      digitalWrite(STEP_PIN, LOW);
    }
  }

  // Pneumatic
  pinMode(pneumaticPin, OUTPUT);
  digitalWrite(pneumaticPin, LOW);

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
    (abs(counter) + abs(counter1) + abs(counter2) + abs(counter3)) / 3; 

  Serial.print("FL=");     Serial.print(counter1);
  Serial.print(" FR=");    Serial.print(counter2);
  Serial.print(" RR=");    Serial.print(counter);
  Serial.print(" RL=");    Serial.print(counter3);
  Serial.print(" AVG=");   Serial.print(avgCounts);
  Serial.print(" YAW=");   Serial.print(currentYaw);
  Serial.print(" ERR=");   Serial.print(headingError());
  Serial.print(" PROX=");  Serial.print(digitalRead(proxPin));
  Serial.print(" STATE="); Serial.println(state);

  // =====================================
  // STATE 0 — STRAFE LEFT (leftCounts1)
  // =====================================
  if (state == 0)
  {
    if (avgCounts < leftCounts1)
      strafeLeft(50);
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
  // Proximity sensor ignored entirely.
  // =====================================
  else if (state == 1)
  {
    if (avgCounts < blindForwardCounts)
      moveForward(40);
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
  // Found  → state 4 (small strafe left)
  // Done   → state 3 (backward scan)
  // =====================================
  else if (state == 2)
  {
    if (digitalRead(proxPin) == LOW)
    {
      stopRobot();
      Serial.println("SPEARHEAD FOUND (forward pass)");
      resetEncoders();
      state = 4;
    }
    else if (avgCounts < forwardCounts)
      moveForward(40);
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
  // Found  → state 4 (small strafe left)
  // Done   → state 13  (not found, stop)
  // =====================================
  else if (state == 3)
  {
    if (digitalRead(proxPin) == LOW)
    {
      stopRobot();
      Serial.println("SPEARHEAD FOUND (backward pass)");
      resetEncoders();
      state = 4;
    }
    else if (avgCounts < forwardCounts)
      moveBackward(40);
    else
    {
      stopRobot();
      Serial.println("SPEARHEAD NOT FOUND");
      state = 14;
    }
  }

  // =====================================
  // STATE 4 — SMALL STRAFE LEFT (leftCounts2)
  // =====================================
  else if (state == 4)
  {
    if (avgCounts < leftCounts2)
      strafeLeft(50);
    else
    {
      stopRobot();
      resetEncoders();
      delay(500);
      state = 5;
    }
  }

  // =====================================
  // STATE 5 — PNEUMATIC GRIP ON
  // Re-check prox after grip.
  // Lost → pneumatic OFF → state 10 (undo strafe) → state 2
  // =====================================
  else if (state == 5)
  {
    stopRobot();
    delay(1000);

    digitalWrite(pneumaticPin, HIGH);
    Serial.println("PNEUMATIC ON");
    delay(500);

    if (digitalRead(proxPin) == LOW)
    {
      Serial.println("STATE 5: prox confirmed — proceeding to state 6");
      state = 6;
    }
    else
    {
      Serial.println("STATE 5: prox lost — PNEUMATIC OFF, undoing strafe (state 10)");
      digitalWrite(pneumaticPin, LOW);
      delay(300);
      resetEncoders();
      state = 10;
    }
  }

  // =====================================
  // STATE 6 — KICK OFF STEPPER FORWARD
  // Immediately transitions to state 7
  // where the non-blocking tick runs.
  // =====================================
  else if (state == 6)
  {
    stopRobot();
    startStepper(false);   // DIR HIGH → 90° forward
    state = 7;
  }

  // =====================================
  // STATE 7 — TICK STEPPER FORWARD (non-blocking)
  // Done → check prox
  //   Still detected → state 11 (strafe right)
  //   Lost           → state 8 (reverse stepper)
  // =====================================
  else if (state == 7)
  {
    bool done = tickStepper();

    if (done)
    {
      Serial.println("STEPPER FORWARD DONE");
      delay(200);   // brief settle

      if (digitalRead(proxPin) == LOW)
      {
        Serial.println("STATE 7: prox confirmed — proceeding to state 11");
        resetEncoders();
        state = 11;
      }
      else
      {
        Serial.println("STATE 7: prox lost — reversing stepper");
        state = 8;
      }
    }
    // While stepper is still moving, other loop tasks run freely.
  }

  // =====================================
  // STATE 8 — KICK OFF STEPPER BACKWARD (undo)
  // Then transitions to state 9.
  // =====================================
  else if (state == 8)
  {
    startStepper(false);   // DIR LOW → reverse 90° back
    state = 9;
  }

  // =====================================
  // STATE 9 — TICK STEPPER BACKWARD (non-blocking)
  // Done → pneumatic OFF → state 10 (undo strafe) → state 2
  // =====================================
  else if (state == 9)
  {
    bool done = tickStepper();

    if (done)
    {
      Serial.println("STEPPER REVERSED DONE");
      delay(200);
      digitalWrite(pneumaticPin, LOW);
      Serial.println("PNEUMATIC OFF — undoing strafe (state 10)");
      delay(300);
      resetEncoders();
      state = 10;
    }
  }

  // =====================================
  // STATE 10 — UNDO STATE 4 (strafe right leftCounts2)
  // Reverses the small strafe left done in state 4.
  // Then goes to state 2 to re-search.
  // =====================================
  else if (state == 10)
  {
    if (avgCounts < leftCounts2)
      strafeRight(50);
    else
    {
      stopRobot();
      resetEncoders();
      delay(500);
      Serial.println("STATE 10: undo strafe done — returning to state 2");
      state = 2;
    }
  }

  // =====================================
  // STATE 11 — STRAFE RIGHT
  // =====================================
  else if (state == 11)
  {
    long strafeCounts =
      (abs(counter) + abs(counter1) + abs(counter2) + abs(counter3)) / 4;

    if (strafeCounts < diagonalCounts)
      strafeRight(50);
    else
    {
      stopRobot();
      resetEncoders();
      delay(500);
      Serial.println("STRAFE RIGHT COMPLETE");
      state = 12;
    }
  }

  // =====================================
  // STATE 12 — ROTATE 90°
  // Heading lock DISABLED — pure rotation
  // =====================================
  else if (state == 12)
  {
    if (avgCounts < turn90Counts)
      rotateRight(50);
    else
    {
      stopRobot();
      releaseStartTime = millis();
      timerStarted     = true;
      holdInitialized  = false;   // reset so hold captures fresh position
      Serial.println("TURN 90 COMPLETE");
      state = 13;
    }
  }

  // =====================================
  // STATE 13 — WAIT 30s WITH ENCODER POSITION HOLD
  // Bot actively resists being pushed.
  // After 30s: pneumatic OFF → state 14.
  // =====================================
  else if (state == 13)
  {
    // if (!holdInitialized)
    // {
    //   holdRR          = counter;
    //   holdFL          = counter1;
    //   holdFR          = counter2;
    //   holdRL          = counter3;
    //   holdInitialized = true;
    //   Serial.println("STATE 13: position hold initialised");
    // }

    // holdPosition();   // fights any external push every iteration

    if (timerStarted && millis() - releaseStartTime >= 30000)
    {
      stopRobot();
      digitalWrite(pneumaticPin, LOW);
      Serial.println("PNEUMATIC OFF");
      state = 14;
    }
  }

  // =====================================
  // STATE 14 — STOP (FINISHED)
  // =====================================
  else if (state == 14)
  {
    stopRobot();
  }

  delay(10);
}

// =====================================
// MOVE BACKWARD — P heading lock
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
// HOLD POSITION — P encoder control
// Each wheel driven independently toward
// its saved target count.
// =====================================
void holdPosition()
{
  int errRR = constrain((int)(kP_pos * (holdRR - counter)),  -maxHoldPWM, maxHoldPWM);
  int errFL = constrain((int)(kP_pos * (holdFL - counter1)), -maxHoldPWM, maxHoldPWM);
  int errFR = constrain((int)(kP_pos * (holdFR - counter2)), -maxHoldPWM, maxHoldPWM);
  int errRL = constrain((int)(kP_pos * (holdRL - counter3)), -maxHoldPWM, maxHoldPWM);

  // RR — forward = wrr_dir LOW
  digitalWrite(wrr_dir_pin, errRR >= 0 ? LOW : HIGH);
  analogWrite(wrr_pwm_pin, abs(errRR));

  // FL — forward = wfl_dir LOW
  digitalWrite(wfl_dir_pin, errFL >= 0 ? LOW : HIGH);
  analogWrite(wfl_pwm_pin, abs(errFL));

  // FR — forward = wfr_dir HIGH
  digitalWrite(wfr_dir_pin, errFR >= 0 ? HIGH : LOW);
  analogWrite(wfr_pwm_pin, abs(errFR));

  // RL — forward = wrl_dir LOW
  digitalWrite(wrl_dir_pin, errRL >= 0 ? LOW : HIGH);
  analogWrite(wrl_pwm_pin, abs(errRL));
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
