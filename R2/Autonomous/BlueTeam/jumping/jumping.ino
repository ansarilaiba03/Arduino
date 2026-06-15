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
// ULTRASONIC SENSOR
// Facing ground — stops BLDC at 50cm
// =====================================
const int trig_pin = 35;
const int echo_pin = 37;
const float BLDC_STOP_DISTANCE_CM = 60.0;

// =====================================
// BLDC + RELAY
// =====================================
Servo bldc;
const int bldc_pin   = 13;
const int relay_pin  = 12;

const int BLDC_STOP  = 1000;
const int BLDC_SPEED = 1800;   

// =====================================
// PNEUMATICS
// =====================================
const int front_pneu_pin = 45;
const int rear_pneu_pin  = 47;

// =====================================
// IMU
// =====================================
float currentYaw         = 0.0;
float desiredHeading     = 0.0;
bool  headingInitialized = false;
const float kP_heading   = 5.5;
const int   maxCorrection = 30;

// =====================================
// ENCODER COUNT TARGETS — TUNE THESE
// =====================================
const long COUNTS_20CM = 200; 
const long COUNTS_30CM = 300; 

// =====================================
// STATES
// =====================================
enum MEGA_State
{
  OPEN_PNEUMATICS_AND_START_BLDC,  // State 0: open pneumatics + start BLDC together
  WAIT_FOR_BLDC_HEIGHT,            // State 1: wait until ultrasonic reads 50cm
  MOVE_FRONT_20CM_FIRST,           // State 2: front wheels forward 20 cm
  CLOSE_FRONT_PNEUMATICS,          // State 3: retract front pneumatics
  MOVE_FRONT_20CM_SECOND,          // State 4: front wheels forward 20 cm
  CLOSE_REAR_PNEUMATICS,           // State 5: retract rear pneumatics
  MOVE_FRONT_30CM_THIRD,           // State 6: front wheels forward 30 cm
  FLIP_RELAY_AND_BLDC_DOWN,        // State 7: flip relay, BLDC anticlockwise back down
  MOVE_FRONT_20CM_FINAL,           // State 8: front wheels final 20 cm — fully on block
  DONE                             // State 9: stop everything
};

MEGA_State megaState = OPEN_PNEUMATICS_AND_START_BLDC;

// =====================================
// ULTRASONIC READ
// Returns distance in cm
// =====================================
float readUltrasonic()
{
  digitalWrite(trig_pin, LOW);  // Clear the trigPin
  delayMicroseconds(2);

  digitalWrite(trig_pin, HIGH); // Set the trigPin on HIGH state for 10 micro seconds
  delayMicroseconds(10);
  digitalWrite(trig_pin, LOW);

   // Read the echoPin, returns the sound wave travel time in microseconds
  long duration = pulseIn(echo_pin, HIGH, 30000);  // 30ms timeout

  // Calculate the distance (cm)
  // Speed of sound is 343m/s or 0.0343 cm/us
  float distance = duration * 0.0343 / 2.0;
  return distance;
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
  return (abs(counter) + abs(counter1) + abs(counter2) + abs(counter3)) / 4;
}

// =====================================
// SETUP
// =====================================
void setup()
{
  Serial.begin(115200);

  // BNO055
  if (!bno.begin()) { Serial.println("BNO055 NOT FOUND"); while (1); }
  bno.setExtCrystalUse(true);
  delay(1000);

  // Motor pins
  pinMode(wfr_dir_pin, OUTPUT); pinMode(wfl_dir_pin, OUTPUT);
  pinMode(wrr_dir_pin, OUTPUT); pinMode(wrl_dir_pin, OUTPUT);
  pinMode(wfr_pwm_pin, OUTPUT); pinMode(wfl_pwm_pin, OUTPUT);
  pinMode(wrr_pwm_pin, OUTPUT); pinMode(wrl_pwm_pin, OUTPUT);

  // Servos
  gripperServo.attach(gripperServoPin);
  rotateServo.attach(rotateServoPin);
  gripperServo.write(90);
  rotateServo.write(0);

  // Ultrasonic
  pinMode(trig_pin, OUTPUT);
  pinMode(echo_pin, INPUT);

  // BLDC
  bldc.attach(bldc_pin);

  Serial.println("Arming ESC...");
  bldc.writeMicroseconds(1000);  // minimum throttle
  delay(5000);                   // wait for ESC beeps

  Serial.println("Starting motor...");
  // Relay
  pinMode(relay_pin, OUTPUT);
  digitalWrite(relay_pin, LOW);

  // Pneumatics
  pinMode(front_pneu_pin, OUTPUT);
  pinMode(rear_pneu_pin,  OUTPUT);
  digitalWrite(front_pneu_pin, LOW);
  digitalWrite(rear_pneu_pin,  LOW);

  // Encoders
  pinMode(outputA,  INPUT_PULLUP); pinMode(outputB,  INPUT_PULLUP);
  pinMode(outputA1, INPUT_PULLUP); pinMode(outputB1, INPUT_PULLUP);
  pinMode(outputA2, INPUT_PULLUP); pinMode(outputB2, INPUT_PULLUP);
  pinMode(outputA3, INPUT_PULLUP); pinMode(outputB3, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(outputA),  readEncoderA,  RISING);
  attachInterrupt(digitalPinToInterrupt(outputA1), readEncoderA1, RISING);
  attachInterrupt(digitalPinToInterrupt(outputA2), readEncoderA2, RISING);
  attachInterrupt(digitalPinToInterrupt(outputA3), readEncoderA3, RISING);

  // // ESC arming delay
  // delay(3000);
  // resetEncoders();
  // Serial.println("System ready");
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

  float dist = readUltrasonic();

  Serial.print("FL=");       Serial.print(counter1);
  Serial.print(" FR=");      Serial.print(counter2);
  Serial.print(" frontAvg=");Serial.print(frontAvg());
  Serial.print(" rearAvg=");Serial.print(rearAvg());
  Serial.print(" allAvg=");Serial.print(allAvg());
  Serial.print(" DIST=");    Serial.print(dist);
  Serial.print(" YAW=");     Serial.print(currentYaw);
  Serial.print(" STATE=");   Serial.println(megaState);

  // =====================================
  // STATE: OPEN_PNEUMATICS_AND_START_BLDC
  // Opens both pneumatics and starts BLDC
  // at the exact same time
  // Then moves to wait for height
  // =====================================
  if (megaState == OPEN_PNEUMATICS_AND_START_BLDC)
  {
    stopRobot();

    // Open both pneumatics
    digitalWrite(front_pneu_pin, HIGH);
    digitalWrite(rear_pneu_pin,  HIGH);
    Serial.println("All pneumatics OPEN");

    // Start BLDC clockwise at same time
    bldc.writeMicroseconds(BLDC_SPEED);
    Serial.println("BLDC CW started");

    megaState = WAIT_FOR_BLDC_HEIGHT;
  }

  // =====================================
  // STATE: WAIT_FOR_BLDC_HEIGHT
  // BLDC is running, lifting front wheels
  // Ultrasonic checks distance to ground
  // Stops BLDC when gap reaches 50cm
  // =====================================
  else if (megaState == WAIT_FOR_BLDC_HEIGHT)
  {
    if (dist >= BLDC_STOP_DISTANCE_CM)
    {
      bldc.writeMicroseconds(BLDC_STOP);
      Serial.println("50cm height reached — BLDC stopped");
      resetEncoders();
      megaState = MOVE_FRONT_20CM_FIRST;
    }
    // else BLDC keeps running — nothing to do here
  }

  // =====================================
  // STATE: MOVE_FRONT_20CM_FIRST
  // Rear wheels move forward 20 cm
  // Front wheels are stopped
  // =====================================
  else if (megaState == MOVE_FRONT_20CM_FIRST)
  {
    if (frontAvg() < COUNTS_20CM)
    {
      moveRearForward(80);
    }
    else
    {
      stopRobot();
      Serial.println("1st 20cm done");
      resetEncoders();
      megaState = CLOSE_FRONT_PNEUMATICS;
    }
  }

  // =====================================
  // STATE: CLOSE_FRONT_PNEUMATICS
  // Front wheels are now on the block
  // Retract front pneumatics
  // =====================================
  else if (megaState == CLOSE_FRONT_PNEUMATICS)
  {
    digitalWrite(front_pneu_pin, LOW);
    Serial.println("Front pneumatics CLOSED");
    delay(300);  // give solenoid time to retract
    resetEncoders();
    megaState = MOVE_FRONT_20CM_SECOND;
  }

  // =====================================
  // STATE: MOVE_FRONT_20CM_SECOND
  // Rear wheels move forward 20 more cm
  // =====================================
  else if (megaState == MOVE_FRONT_20CM_SECOND)
  {
    if (frontAvg() < COUNTS_20CM)
    {
      moveRearForward(80);
    }
    else
    {
      stopRobot();
      Serial.println("2nd 20cm done");
      resetEncoders();
      megaState = CLOSE_REAR_PNEUMATICS;
    }
  }

  // =====================================
  // STATE: CLOSE_REAR_PNEUMATICS
  // Retract rear pneumatics
  // =====================================
  else if (megaState == CLOSE_REAR_PNEUMATICS)
  {
    digitalWrite(rear_pneu_pin, LOW);
    Serial.println("Rear pneumatics CLOSED");
    delay(300);
    resetEncoders();
    megaState = MOVE_FRONT_30CM_THIRD;
  }

  // =====================================
  // STATE: MOVE_FRONT_30CM_THIRD
  // All wheels move forward 30 more cm
  // =====================================
  else if (megaState == MOVE_FRONT_30CM_THIRD)
  {
    if (frontAvg() < COUNTS_30CM)
    {
      moveAllForward(80);
    }
    else
    {
      stopRobot();
      Serial.println("3rd 30cm done");
      resetEncoders();
      megaState = DONE;
      //megaState = FLIP_RELAY_AND_BLDC_DOWN;
    }
  }

  // =====================================
  // STATE: FLIP_RELAY_AND_BLDC_DOWN
  // Relay flips to change BLDC phase
  // BLDC now runs anticlockwise
  // Lowers rear wheels onto block
  // Waits until ultrasonic reads < 50cm
  // (gap closing as robot lowers)
  // =====================================
  else if (megaState == FLIP_RELAY_AND_BLDC_DOWN)
  {
    // Flip relay to reverse BLDC phase
    digitalWrite(relay_pin, HIGH);
    delay(200);  // relay switching time

    // Start BLDC anticlockwise
    bldc.writeMicroseconds(BLDC_SPEED);
    Serial.println("Relay flipped — BLDC CCW started");

    // Wait until robot has lowered back (distance drops below threshold)
    while (dist >= 5.0)  // tune: stops when nearly touching ground again
    {
      dist = readUltrasonic();
      Serial.print("Lowering... DIST="); Serial.println(dist);
      delay(20);
    }

    bldc.writeMicroseconds(BLDC_STOP);
    digitalWrite(relay_pin, LOW);
    Serial.println("BLDC down — rear wheels on block");

    resetEncoders();
    megaState = MOVE_FRONT_20CM_FINAL;
  }

  // =====================================
  // STATE: MOVE_FRONT_20CM_FINAL
  // Final 20 cm — robot fully on block
  // =====================================
  else if (megaState == MOVE_FRONT_20CM_FINAL)
  {
    if (frontAvg() < COUNTS_20CM)
    {
      moveFrontForward(80);
    }
    else
    {
      stopRobot();
      Serial.println("Final 20cm done — fully on block!");
      megaState = DONE;
    }
  }

  // =====================================
  // STATE: DONE
  // Stop everything
  // =====================================
  else if (megaState == DONE)
  {
    stopRobot();
    bldc.writeMicroseconds(BLDC_STOP);
    digitalWrite(front_pneu_pin, LOW);
    digitalWrite(rear_pneu_pin,  LOW);
    digitalWrite(relay_pin,      LOW);
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
