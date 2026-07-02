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
// PROXIMITY SENSORS
// =====================================
const int proxPin = 33;
const int proxPin2 = 31;

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

// RR
int outputA = 18;
int outputB = 41;

// FL
int outputA1 = 19;
int outputB1 = 45;

// FR
int outputA2 = 2;
int outputB2 = 49;

// RL
int outputA3 = 3;
int outputB3 = 53;

// =====================================
// COUNTERS
// =====================================
volatile long counter = 0;
volatile long counter1 = 0;
volatile long counter2 = 0;
volatile long counter3 = 0;

// =====================================
// DISTANCES (TUNE THESE)
// =====================================
long leftCounts    = 700;   // strafe left
long forwardCounts = 560;   // move forward
long stepCounts    = 200;    // 20 cm step
long diagonalCounts = 700;   // tune experimentally
long turn90Counts   = 600;   // tune experimentally

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

  gripperServo.attach(gripperServoPin);
  rotateServo.attach(rotateServoPin);

  gripperServo.write(180);
  rotateServo.write(40);

  pinMode(proxPin, INPUT);
  pinMode(proxPin2, INPUT);

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
    (abs(counter) + abs(counter1)+abs(counter2) + abs(counter3)) / 4;

  
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
      strafeLeft(80);
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

      delay(1000);
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
      moveForward(80);
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

      delay(1000);
      state = 2;
    }
  }

  // =====================================
  // STATE 2
  // CHECK PROXIMITY
  // =====================================

  else if(state == 2)
  {
    if(digitalRead(proxPin) == LOW)
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

    Serial.println("SPEARHEAD FOUND");

    state = 4;
  }
    else
    {
      if(searchCount >= 5)
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

        Serial.println("SPEARHEAD NOT FOUND");

        state = 99;
      }
      else
      {
        resetEncoders();
        Serial.print("After Reset: ");
        Serial.print(counter);
        Serial.print(" ");
        Serial.print(counter1);
        Serial.print(" ");
        Serial.print(counter2);
        Serial.print(" ");
        Serial.println(counter3);

      delay(1000);
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
      moveForward(80);
    }
    else
    {
      stopRobot();

      searchCount++;

      resetEncoders();
      Serial.print("After Reset: ");
      Serial.print(counter);
      Serial.print(" ");
      Serial.print(counter1);
      Serial.print(" ");
      Serial.print(counter2);
      Serial.print(" ");
      Serial.println(counter3);

      delay(1000);

      state = 2;
    }
  }

  // =====================================
// STATE 4
// CLOSE GRIPPER
// =====================================

else if(state == 4)
{
  stopRobot();

  gripperServo.write(0);

  Serial.println("GRIPPER CLOSED");

  delay(1000);

  state = 5;
}

// =====================================
// STATE 5
// ROTATE SPEARHEAD
// =====================================

else if(state == 5)
{
  rotateServo.write(130);

  Serial.println("ROTATE SERVO TO 180");

  delay(1000);

  state = 61;
}

// =====================================
// STATE 6
// WAIT FOR SENSOR 2
// =====================================

else if(state == 6)
{
  if(digitalRead(proxPin2) == LOW)
  {
    Serial.println("SECOND SENSOR DETECTED");

    resetEncoders();

    state = 61;
  }
}

// =====================================
// STATE 61
// RIGHT HORIZONTAL 130 mm
// =====================================

else if(state == 61)
{
  long diagCounts =
    (abs(counter) + abs(counter1) + abs(counter2) + abs(counter3)) / 4;

  if(diagCounts < diagonalCounts)
  {
    moveRight(80);
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

      delay(1000);

    Serial.println("HORIZONTAL COMPLETE");

    state = 62;
  }
}

// =====================================
// STATE 62
// ROTATE 90 DEG
// =====================================

else if(state == 62)
{
  if(avgCounts < turn90Counts)
  {
    rotateLeft(80);
  }
  else
  {
    stopRobot();

    releaseStartTime = millis();
    timerStarted = true;

    Serial.println("TURN COMPLETE");

    state = 7;
  }
}

// =====================================
// STATE 7
// WAIT 30 SECONDS
// =====================================

else if(state == 7)
{
  if(timerStarted &&
     millis() - releaseStartTime >= 30000)
  {
    gripperServo.write(180);

    Serial.println("GRIPPER OPENED");

    state = 99;
  }
}

// =====================================
// STATE 8
// WAIT UNTIL SENSOR 2 IS LOW
// =====================================

else if(state == 8)
{
  if(digitalRead(proxPin2) == LOW)
  {
    rotateServo.write(90);

    Serial.println("ROTATE SERVO RESET");

    state = 99;
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

// gradual stop

void gradualStop(int startPWM)
{
  for(int pwm = startPWM; pwm >= 0; pwm -= 5)
  {
    analogWrite(wfr_pwm_pin, pwm);
    analogWrite(wfl_pwm_pin, pwm);
    analogWrite(wrr_pwm_pin, pwm);
    analogWrite(wrl_pwm_pin, pwm);

    delay(20);
  }

  analogWrite(wfr_pwm_pin, 0);
  analogWrite(wfl_pwm_pin, 0);
  analogWrite(wrr_pwm_pin, 0);
  analogWrite(wrl_pwm_pin, 0);
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
// HORIZONTAL RIGHT
// =====================================

void moveRight(int pwm)
{
  
  digitalWrite(wfl_dir_pin, LOW); //forward

  
  digitalWrite(wrr_dir_pin, LOW); //forward
  digitalWrite(wfr_dir_pin, LOW); //backward
  digitalWrite(wrl_dir_pin, HIGH);  //backward


  analogWrite(wfl_pwm_pin, pwm);
  analogWrite(wrr_pwm_pin, pwm);

  analogWrite(wfr_pwm_pin, pwm);
  analogWrite(wrl_pwm_pin, pwm);
}

// =====================================
// ROTATE LEFT
// =====================================

void rotateLeft(int pwm)
{
  // FR CW
  digitalWrite(wfr_dir_pin, HIGH);

  // RR CW
  digitalWrite(wrr_dir_pin, LOW);

  // FL ACW
  digitalWrite(wfl_dir_pin, HIGH);

  // RL AcW
  digitalWrite(wrl_dir_pin, HIGH);

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