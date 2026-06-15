#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// =====================================
// BNO055 — SDA=20, SCL=21
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
// SERVOS
// =====================================
Servo gripperServo;
Servo rotateServo;
const int gripperServoPin = 10;
const int rotateServoPin  = 9;

// =====================================
// ENCODERS
// =====================================
const int outputA  = 18; const int outputB  = 48;  // RR
const int outputA1 = 19; const int outputB1 = 31;  // FL
const int outputA2 = 2;  const int outputB2 = 30;  // FR
const int outputA3 = 3;  const int outputB3 = 22;  // RL

volatile long counter  = 0;  // RR
volatile long counter1 = 0;  // FL
volatile long counter2 = 0;  // FR
volatile long counter3 = 0;  // RL

// =====================================
// IMU
// =====================================
float currentYaw         = 0.0;
float desiredHeading     = 0.0;
bool  headingInitialized = false;
const float kP_heading   = 5.5;
const int   maxCorrection = 30;

// =====================================
// UART TO ESP32-S3
// Mega TX3=14, RX3=15
// =====================================
const uint8_t CMD_OPEN_ALL_PNEU    = 0x01;
const uint8_t CMD_CLOSE_FRONT_PNEU = 0x02;
const uint8_t CMD_CLOSE_REAR_PNEU  = 0x03;
const uint8_t CMD_BLDC_CCW         = 0x04;

const uint8_t ACK_PNEU_OPENED    = 0x11;
const uint8_t ACK_BLDC_UP_DONE   = 0x12;
const uint8_t ACK_FRONT_CLOSED   = 0x13;
const uint8_t ACK_REAR_CLOSED    = 0x14;
const uint8_t ACK_BLDC_DOWN_DONE = 0x15;

// =====================================
// ENCODER COUNT TARGETS 
// =====================================
const long COUNTS_20CM = 200; 
const long COUNTS_30CM = 300;  


// =====================================
// STATES
// =====================================
enum MEGA_State
{
  MEGA_OPEN_PNEUMATICS_AND_BLDC_UP,  // State 0: send command, wait for ACKs
  MEGA_MOVE_20CM_FIRST,              // State 1: front wheels forward 10 cm
  MEGA_CLOSE_FRONT_PNEUMATICS,       // State 2: retract front pneumatics
  MEGA_MOVE_20CM_SECOND,             // State 3: front wheels forward 10 cm
  MEGA_CLOSE_REAR_PNEUMATICS,        // State 4: retract rear pneumatics
  MEGA_MOVE_30CM_THIRD,              // State 5: front wheels forward 10 cm
  MEGA_BLDC_CCW,                     // State 6: flip phase, BLDC down
  MEGA_MOVE_20CM_FINAL,              // State 7: front wheels final 10 cm
  MEGA_DONE,                         // State 8: stop everything
  MEGA_ERROR                         // State 9: ACK timeout error
};

MEGA_State megaState = MEGA_OPEN_PNEUMATICS_AND_BLDC_UP;

// =====================================
// HELPER: WAIT FOR ACK (10s timeout)
// =====================================
bool waitForACK(uint8_t expectedACK)
{
  unsigned long start = millis();
  while (millis() - start < 10000)
  {
    if (Serial3.available())
    {
      uint8_t ack = Serial3.read();
      Serial.print("ACK: 0x"); Serial.println(ack, HEX);
      if (ack == expectedACK) return true;
    }
  }
  Serial.println("ACK TIMEOUT");
  return false;
}

// =====================================
// avg
// =====================================
long frontAvg()
{
  return (abs(counter1) + abs(counter2)) / 2;
}

long rearAvg()
{
  return (abs(counter) + abs(counter3)) / 2;
}

long allAvg()
{
  return (abs(counter) abs(counter1) + abs(counter2) + abs(counter3)) / 4;
}

// =====================================
// IMU FUNCTIONS
// =====================================
void readIMU()
{
  imu::Quaternion quat = bno.getQuat();
  double w = quat.w(), x = quat.x(), y = quat.y(), z = quat.z();
  double sinYaw = 2.0 * (w * z + x * y);
  double cosYaw = 1.0 - 2.0 * (y * y + z * z);
  currentYaw = (float)(atan2(sinYaw, cosYaw) * 180.0 / PI);
}

float headingError()
{
  float err = currentYaw - desiredHeading;
  if (err >  180.0) err -= 360.0;
  if (err < -180.0) err += 360.0;
  return err;
}

int getCorrection()
{
  return constrain((int)(kP_heading * headingError()), -maxCorrection, maxCorrection);
}

// =====================================
// MOVEMENT FUNCTIONS
// =====================================
void stopRobot()
{
  analogWrite(wfr_pwm_pin, 0);
  analogWrite(wfl_pwm_pin, 0);
  analogWrite(wrr_pwm_pin, 0);
  analogWrite(wrl_pwm_pin, 0);
}

// Front 2 wheels only — rear stopped
void moveFrontForward(int pwm)
{
  int corr     = getCorrection();
  int pwmLeft  = constrain(pwm + corr, 0, 255);
  int pwmRight = constrain(pwm - corr, 0, 255);

  digitalWrite(wfr_dir_pin, HIGH);
  digitalWrite(wfl_dir_pin, LOW);

  analogWrite(wfr_pwm_pin, pwmRight);
  analogWrite(wfl_pwm_pin, pwmLeft);
  analogWrite(wrr_pwm_pin, 0);   // rear stopped
  analogWrite(wrl_pwm_pin, 0);
}

// rear 2 wheels only — front stopped
void moveRearForward(int pwm)
{
  int corr     = getCorrection();
  int pwmLeft  = constrain(pwm + corr, 0, 255);
  int pwmRight = constrain(pwm - corr, 0, 255);

  digitalWrite(wrr_dir_pin, LOW);
  digitalWrite(wrl_dir_pin, LOW);

  analogWrite(wfr_pwm_pin, pwmRight);
  analogWrite(wfl_pwm_pin, pwmLeft);
  analogWrite(wfr_pwm_pin, 0);   
  analogWrite(wfl_pwm_pin, 0);
}

// All 4
void moveAllForward(int pwm)
{
  int corr     = getCorrection();
  int pwmLeft  = constrain(pwm + corr, 0, 255);
  int pwmRight = constrain(pwm - corr, 0, 255);

  digitalWrite(wfr_dir_pin, HIGH);
  digitalWrite(wfl_dir_pin, LOW);
  digitalWrite(wrr_dir_pin, LOW);
  digitalWrite(wrl_dir_pin, LOW);

  analogWrite(wfr_pwm_pin, pwmRight);
  analogWrite(wfl_pwm_pin, pwmLeft);
  analogWrite(wrr_pwm_pin, pwmRight);   
  analogWrite(wrl_pwm_pin, pwmLeft);
}

void resetEncoders()
{
  counter = 0; counter1 = 0; counter2 = 0; counter3 = 0;
}

// =====================================
// SETUP
// =====================================
void setup()
{
  Serial.begin(115200);
  Serial3.begin(115200);   // UART to ESP32: TX=14, RX=15

  if (!bno.begin()) { Serial.println("BNO055 NOT FOUND"); while (1); }
  bno.setExtCrystalUse(true);
  delay(1000);

  pinMode(wfr_dir_pin, OUTPUT); pinMode(wfl_dir_pin, OUTPUT);
  pinMode(wrr_dir_pin, OUTPUT); pinMode(wrl_dir_pin, OUTPUT);
  pinMode(wfr_pwm_pin, OUTPUT); pinMode(wfl_pwm_pin, OUTPUT);
  pinMode(wrr_pwm_pin, OUTPUT); pinMode(wrl_pwm_pin, OUTPUT);

  gripperServo.attach(gripperServoPin);
  rotateServo.attach(rotateServoPin);
  gripperServo.write(90);
  rotateServo.write(0);

  pinMode(outputA,  INPUT_PULLUP); pinMode(outputB,  INPUT_PULLUP);
  pinMode(outputA1, INPUT_PULLUP); pinMode(outputB1, INPUT_PULLUP);
  pinMode(outputA2, INPUT_PULLUP); pinMode(outputB2, INPUT_PULLUP);
  pinMode(outputA3, INPUT_PULLUP); pinMode(outputB3, INPUT_PULLUP);

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
    Serial.print("Heading locked: "); Serial.println(desiredHeading);
  }

  Serial.print("FL=");       Serial.print(counter1);
  Serial.print(" FR=");      Serial.print(counter2);
  Serial.print(" frontAvg=");Serial.print(frontAvg());
  Serial.print(" rearAvg=");Serial.print(rearAvg());
  Serial.print(" allAvg=");Serial.print(allAvg());
  Serial.print(" YAW=");     Serial.print(currentYaw);
  Serial.print(" STATE=");   Serial.println(megaState);

  // =====================================
  // STATE: MEGA_OPEN_PNEUMATICS_AND_BLDC_UP
  // Sends one command to ESP32
  // Waits for one single ACK that confirms:
  //   - pneumatics are open
  //   - BLDC has reached 500 counts
  // Both happened simultaneously on ESP32
  // =====================================
  if (megaState == MEGA_OPEN_PNEUMATICS_AND_BLDC_UP)
  {
    stopRobot();
    Serial.println("Sending: Open pneumatics + BLDC CW simultaneously");
    Serial3.write(CMD_OPEN_ALL_PNEU);

    if (!waitForACK(ACK_READY)) { megaState = MEGA_ERROR; return; }
    Serial.println("Pneumatics open + BLDC at 500 counts confirmed");

    resetEncoders();
    megaState = MEGA_MOVE_20CM_FIRST;
  }

  // =====================================
  // STATE: MOVE FRONT WHEELS 20 CM (1st)
  // After BLDC lifted front wheels
  // =====================================
  else if (megaState == MEGA_MOVE_20CM_FIRST)
  {
    if (frontAvg() < COUNTS_20CM)
    {
      moveFrontForward(80);
    }
    else
    {
      stopRobot();
      Serial.println("1st 10 cm done");
      resetEncoders();
      megaState = MEGA_CLOSE_FRONT_PNEUMATICS;
    }
  }

  // =====================================
  // STATE: CLOSE FRONT PNEUMATICS
  // Front wheels now touching block
  // =====================================
  else if (megaState == MEGA_CLOSE_FRONT_PNEUMATICS)
  {
    Serial.println("Sending: Close front pneumatics");
    Serial3.write(CMD_CLOSE_FRONT_PNEU);

    if (!waitForACK(ACK_FRONT_CLOSED)) { megaState = MEGA_ERROR; return; }
    Serial.println("Front pneumatics closed confirmed");

    resetEncoders();
    megaState = MEGA_MOVE_10CM_SECOND;
  }

  // =====================================
  // STATE: MOVE FRONT WHEELS 10 CM (2nd)
  // =====================================
  else if (megaState == MEGA_MOVE_10CM_SECOND)
  {
    if (frontAvg() < COUNTS_10CM)
    {
      moveFrontForward(80);
    }
    else
    {
      stopRobot();
      Serial.println("2nd 10 cm done");
      resetEncoders();
      megaState = MEGA_CLOSE_REAR_PNEUMATICS;
    }
  }

  // =====================================
  // STATE: CLOSE REAR PNEUMATICS
  // =====================================
  else if (megaState == MEGA_CLOSE_REAR_PNEUMATICS)
  {
    Serial.println("Sending: Close rear pneumatics");
    Serial3.write(CMD_CLOSE_REAR_PNEU);

    if (!waitForACK(ACK_REAR_CLOSED)) { megaState = MEGA_ERROR; return; }
    Serial.println("Rear pneumatics closed confirmed");

    resetEncoders();
    megaState = MEGA_MOVE_10CM_THIRD;
  }

  // =====================================
  // STATE: MOVE FRONT WHEELS 10 CM (3rd)
  // =====================================
  else if (megaState == MEGA_MOVE_10CM_THIRD)
  {
    if (frontAvg() < COUNTS_10CM)
    {
      moveFrontForward(80);
    }
    else
    {
      stopRobot();
      Serial.println("3rd 10 cm done");
      resetEncoders();
      megaState = MEGA_BLDC_CCW;
    }
  }

  // =====================================
  // STATE: BLDC CCW (phase flip)
  // Relay flips, BLDC goes back to 0 counts
  // This lifts rear wheels onto block
  // =====================================
  else if (megaState == MEGA_BLDC_CCW)
  {
    Serial.println("Sending: BLDC CCW (phase flip)");
    Serial3.write(CMD_BLDC_CCW);

    if (!waitForACK(ACK_BLDC_DOWN_DONE)) { megaState = MEGA_ERROR; return; }
    Serial.println("BLDC down confirmed");

    resetEncoders();
    megaState = MEGA_MOVE_10CM_FINAL;
  }

  // =====================================
  // STATE: MOVE FRONT WHEELS FINAL 10 CM
  // Robot fully on block
  // =====================================
  else if (megaState == MEGA_MOVE_10CM_FINAL)
  {
    if (frontAvg() < COUNTS_10CM)
    {
      moveFrontForward(80);
    }
    else
    {
      stopRobot();
      Serial.println("Final 10 cm done — fully on block!");
      megaState = MEGA_DONE;
    }
  }

  // =====================================
  // STATE: DONE
  // =====================================
  else if (megaState == MEGA_DONE)
  {
    stopRobot();
  }

  // =====================================
  // STATE: ERROR
  // ACK timeout from ESP32
  // =====================================
  else if (megaState == MEGA_ERROR)
  {
    stopRobot();
    Serial.println("ERROR — check ESP32 connection");
  }

  delay(10);
}

// =====================================
// ENCODER ISRs
// =====================================
void readEncoderA()
{
  if (digitalRead(outputA)  == digitalRead(outputB))  counter++;  else counter--;
}
void readEncoderA1()
{
  if (digitalRead(outputA1) == digitalRead(outputB1)) counter1++; else counter1--;
}
void readEncoderA2()
{
  if (digitalRead(outputA2) == digitalRead(outputB2)) counter2++; else counter2--;
}
void readEncoderA3()
{
  if (digitalRead(outputA3) == digitalRead(outputB3)) counter3--; else counter3++;
}
