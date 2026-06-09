#include <Arduino.h>

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
const int proxPin = 8;   

// =====================================
// ENCODERS
// =====================================

// RR
int outputA = 18;
int outputB = 48;

// FL
int outputA1 = 19;
int outputB1 = 38;

// FR
int outputA2 = 20;
int outputB2 = 30;

// RL
int outputA3 = 21;
int outputB3 = 22;

// =====================================
// COUNTERS
// =====================================
volatile long counter = 0;
volatile long counter1 = 0;
volatile long counter2 = 0;
volatile long counter3 = 0;

// =====================================
// DISTANCES
// =====================================
long leftCounts    = 750;   // strafe left
long forwardCounts = 500;   // move forward
long stepCounts    = 200;    // 20 cm step

// =====================================
// STATE MACHINE
// =====================================
int state = 0;
int searchCount = 0;

// =====================================
// SETUP
// =====================================
void setup()
{
  Serial.begin(115200);

  pinMode(wfr_dir_pin, OUTPUT);
  pinMode(wfl_dir_pin, OUTPUT);
  pinMode(wrr_dir_pin, OUTPUT);
  pinMode(wrl_dir_pin, OUTPUT);

  pinMode(wfr_pwm_pin, OUTPUT);
  pinMode(wfl_pwm_pin, OUTPUT);
  pinMode(wrr_pwm_pin, OUTPUT);
  pinMode(wrl_pwm_pin, OUTPUT);

  pinMode(proxPin, INPUT);

  pinMode(outputA, INPUT_PULLUP);
  pinMode(outputB, INPUT_PULLUP);

  pinMode(outputA1, INPUT_PULLUP);
  pinMode(outputB1, INPUT_PULLUP);

  pinMode(outputA2, INPUT_PULLUP);
  pinMode(outputB2, INPUT_PULLUP);

  pinMode(outputA3, INPUT_PULLUP);
  pinMode(outputB3, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(outputA), readEncoderA, RISING);
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
  long avgCounts =
    (abs(counter) + abs(counter1) + abs(counter2) + abs(counter3)) / 4;

  
  Serial.print("FL=");
  Serial.print(counter1);
  
  Serial.print(" FR=");
  Serial.print(counter2);
  
  Serial.print(" RR=");
  Serial.print(counter);

  Serial.print(" RL=");
  Serial.print(counter3);

  Serial.print(" AVG=");
  Serial.print(avgCounts);

  Serial.print(" STATE=");
  Serial.println(state);

  // =====================================
  // STATE 0
  // STRAFE LEFT
  // =====================================

  if(state == 0)
  {
    if(avgCounts < leftCounts)
    {
      strafeLeft(120);
    }
    else
    {
      stopRobot();
      resetEncoders();
      Serial.print("After Reset: ");
Serial.print(counter);
Serial.print(" ");
Serial.print(counter1);
Serial.print(" ");
Serial.print(counter2);
Serial.print(" ");
Serial.println(counter3);

      delay(5000);
      state = 1;
    }
  }

  // =====================================
  // STATE 1
  // MOVE FORWARD
  // =====================================

  else if(state == 1)
  {
    if(avgCounts < forwardCounts)
    {
      moveForward(120);
    }
    else
    {
      stopRobot();
      resetEncoders();
      state = 99;
    }
  }

  // =====================================
  // STATE 2
  // CHECK PROXIMITY
  // =====================================

  else if(state == 2)
  {
    if(digitalRead(proxPin) == HIGH)
    {
      stopRobot();

      Serial.println("SPEARHEAD FOUND");

      state = 99;
    }
    else
    {
      if(searchCount >= 5)
      {
        stopRobot();

        Serial.println("SPEARHEAD NOT FOUND");

        state = 99;
      }
      else
      {
        resetEncoders();
        state = 3;
      }
    }
  }

  // =====================================
  // STATE 3
  // MOVE AHEAD
  // =====================================

  else if(state == 3)
  {
    if(avgCounts < stepCounts)
    {
      moveForward(120);
    }
    else
    {
      stopRobot();

      searchCount++;

      resetEncoders();

      state = 2;
    }
  }

  // =====================================
  // FINISHED
  // =====================================

  else if(state == 99)
  {
    stopRobot();
  }

  delay(10);
}

// =====================================
// RESET
// =====================================

void resetEncoders()
{
  counter = 0;
  counter1 = 0;
  counter2 = 0;
  counter3 = 0;
}

// =====================================
// STRAFE LEFT
// =====================================

void strafeLeft(int pwm)
{
  digitalWrite(wfr_dir_pin, HIGH);
  digitalWrite(wrr_dir_pin, HIGH);
  digitalWrite(wfl_dir_pin, HIGH);
  digitalWrite(wrl_dir_pin, LOW);

  analogWrite(wfr_pwm_pin, pwm);
  analogWrite(wrl_pwm_pin, pwm);
  analogWrite(wfl_pwm_pin, pwm);
  analogWrite(wrr_pwm_pin, pwm);
}

// =====================================
// MOVE FORWARD
// =====================================

void moveForward(int pwm)
{
  digitalWrite(wfr_dir_pin, HIGH);
  digitalWrite(wrl_dir_pin, LOW);
  digitalWrite(wfl_dir_pin, LOW);
  digitalWrite(wrr_dir_pin, LOW);

  analogWrite(wfr_pwm_pin, pwm);
  analogWrite(wrl_pwm_pin, pwm);
  analogWrite(wfl_pwm_pin, pwm);
  analogWrite(wrr_pwm_pin, pwm);
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
// ENCODERS
// =====================================

void readEncoderA()
{
  if(digitalRead(outputA) == digitalRead(outputB))
    counter++;
  else
    counter--;
}

void readEncoderA1()
{
  if(digitalRead(outputA1) == digitalRead(outputB1))
    counter1++;
  else
    counter1--;
}

void readEncoderA2()
{
  if(digitalRead(outputA2) == digitalRead(outputB2))
    counter2++;
  else
    counter2--;
}

void readEncoderA3()
{
  if(digitalRead(outputA3) == digitalRead(outputB3))
    counter3--;
  else
    counter3++;
}