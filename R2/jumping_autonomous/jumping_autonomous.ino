#include <Arduino.h>
#include <ESP32Servo.h>

// ===================== PINS =====================

const int WFR_DIR = 27;
const int WFL_DIR = 29;
const int WRR_DIR = 25;
const int WRL_DIR = 23;

const int WFR_PWM = 6;
const int WFL_PWM = 7;
const int WRR_PWM = 5;
const int WRL_PWM = 4;

const int FRONT_PNEU = 11;
const int REAR_PNEU  = 12;

const int RELAY_PIN = 13;
const int BLDC_PIN  = 14;

// ===================== ASSUMED VALUES =====================

const float DIST_1 = 200;   
const float DIST_2 = 200;  
const float DIST_3 = 300;  
const float DIST_4 = 200;  

const float BLDC_UP_POS   = 500;
const float BLDC_DOWN_POS = 0;  

const int DRIVE_PWM = 120;

const int BLDC_FORWARD_SPEED = 1500;
const int BLDC_STOP_SPEED    = 1000;
const int BLDC_REVERSE_SPEED = 1500;

// Encoder conversion factors
const float DRIVE_CM_PER_COUNT = 0.05;
const float BLDC_CM_PER_COUNT  = 0.02;

// =====================================
// ENCODERS
// =====================================

// RR
int outputA = 2;
int outputB = 50;

// FL
int outputA1 = 18;
int outputB1 = 53;

// FR
int outputA2 = 3;
int outputB2 = 52;

// RL
int outputA3 = 19;
int outputB3 = 51;


int bldcA = 20;
int bldcB = 21;

// =====================================
// COUNTERS
// =====================================
volatile long counter = 0;
volatile long counter1 = 0;
volatile long counter2 = 0;
volatile long counter3 = 0;
volatile long bldcEncoder = 0;

// ===================== SERVO =====================

Servo bldc;

// ===================== STATES =====================

enum DriveMode
{
    ALL_WHEELS,
    REAR_ONLY,
    FRONT_ONLY
};

DriveMode driveMode = REAR_ONLY;

enum RobotState
{
    OPEN_AND_LIFT,
    WAIT_BLDC_UP,

    MOVE_20_FIRST,

    RETRACT_FRONT,

    MOVE_20_SECOND,

    RETRACT_REAR,

    MOVE_30,

    REVERSE_BLDC,

    WAIT_BLDC_DOWN,

    MOVE_20_FINAL,

    FINISHED
};

RobotState state = OPEN_AND_LIFT;

// ===================== HELPERS =====================

void resetDriveEncoders()
{
    counter  = 0;
    counter1 = 0;
    counter2 = 0;
    counter3 = 0;
}

float getDriveDistanceCM()
{
    long counts = 0;

    switch(driveMode)
    {
        case REAR_ONLY:

            counts =
            (
                abs(counter) +
                abs(counter3)
            ) / 2;

            break;

        case FRONT_ONLY:

            counts =
            (
                abs(counter1) +
                abs(counter2)
            ) / 2;

            break;

        case ALL_WHEELS:

            counts =
            (
                abs(counter) +
                abs(counter1) +
                abs(counter2) +
                abs(counter3)
            ) / 4;

            break;
    }

    return counts * DRIVE_CM_PER_COUNT;
}

float getBLDCDistanceCM()
{
    return bldcEncoder * BLDC_CM_PER_COUNT;
}

void moveForward()
{
    digitalWrite(WFR_DIR, HIGH);
    digitalWrite(WFL_DIR, HIGH);
    digitalWrite(WRR_DIR, HIGH);
    digitalWrite(WRL_DIR, HIGH);

    analogWrite(WFR_PWM, DRIVE_PWM);
    analogWrite(WFL_PWM, DRIVE_PWM);
    analogWrite(WRR_PWM, DRIVE_PWM);
    analogWrite(WRL_PWM, DRIVE_PWM);
}

void stopDrive()
{
    analogWrite(WFR_PWM, 0);
    analogWrite(WFL_PWM, 0);
    analogWrite(WRR_PWM, 0);
    analogWrite(WRL_PWM, 0);
}

void stopAll()
{
    stopDrive();

    digitalWrite(FRONT_PNEU, LOW);
    digitalWrite(REAR_PNEU, LOW);

    bldc.writeMicroseconds(BLDC_STOP_SPEED);
}

// ===================== SETUP =====================

void setup()
{
    Serial.begin(115200);

    pinMode(WFR_DIR, OUTPUT);
    pinMode(WFL_DIR, OUTPUT);
    pinMode(WRR_DIR, OUTPUT);
    pinMode(WRL_DIR, OUTPUT);

    pinMode(WFR_PWM, OUTPUT);
    pinMode(WFL_PWM, OUTPUT);
    pinMode(WRR_PWM, OUTPUT);
    pinMode(WRL_PWM, OUTPUT);

    pinMode(FRONT_PNEU, OUTPUT);
    pinMode(REAR_PNEU, OUTPUT);

    pinMode(RELAY_PIN, OUTPUT);

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

    pinMode(bldcA, INPUT_PULLUP);
    pinMode(bldcB, INPUT_PULLUP);

    attachInterrupt(
    digitalPinToInterrupt(bldcA),
    readBLDCEncoder,
    RISING
);


    bldc.setPeriodHertz(50);
    bldc.attach(BLDC_PIN, 500, 2400);

    bldc.writeMicroseconds(BLDC_STOP_SPEED);

    delay(3000);

    Serial.println("Autonomous Start");
}


void readBLDCEncoder()
{
    if(digitalRead(bldcB))
        bldcEncoder++;
    else
        bldcEncoder--;
}

void readEncoderA()
{
    if(digitalRead(outputB))
        counter++;
    else
        counter--;
}

void readEncoderA1()
{
    if(digitalRead(outputB1))
        counter1++;
    else
        counter1--;
}

void readEncoderA2()
{
    if(digitalRead(outputB2))
        counter2++;
    else
        counter2--;
}

void readEncoderA3()
{
    if(digitalRead(outputB3))
        counter3++;
    else
        counter3--;
}

// ===================== LOOP =====================

void loop()
{
    Serial.print("FL=");
  Serial.print(counter1);
  
  Serial.print(" FR=");
  Serial.print(counter2);
  
  Serial.print(" RR=");
  Serial.print(counter);

  Serial.print(" RL=");
  Serial.print(counter3);

  Serial.print(" AVG=");
  Serial.print(counts);

  Serial.print(" STATE=");
  Serial.println(state);

    switch(state)
    {
        // ----------------------------------
        case OPEN_AND_LIFT:

            Serial.println("OPEN_AND_LIFT");

            driveMode = REAR_ONLY;

            digitalWrite(FRONT_PNEU, HIGH);
            digitalWrite(REAR_PNEU, HIGH);

            digitalWrite(RELAY_PIN, HIGH);

            bldc.writeMicroseconds(BLDC_FORWARD_SPEED);

            state = WAIT_BLDC_UP;

            break;

        // ----------------------------------
        case WAIT_BLDC_UP:

            if(getBLDCDistanceCM() >= BLDC_UP_POS)
            {
                bldc.writeMicroseconds(BLDC_STOP_SPEED);

                resetDriveEncoders();

                moveForward();

                state = MOVE_20_FIRST;
            }

            break;

        // ----------------------------------
        case MOVE_20_FIRST:

            if(getDriveDistanceCM() >= DIST_1)
            {
                driveMode = REAR_ONLY;

                stopDrive();

                digitalWrite(FRONT_PNEU, LOW);

                resetDriveEncoders();

                moveForward();

                state = MOVE_20_SECOND;
            }

            break;

        // ----------------------------------
        case MOVE_20_SECOND:

            if(getDriveDistanceCM() >= DIST_2)
            {
                driveMode = REAR_ONLY;
                stopDrive();

                digitalWrite(REAR_PNEU, LOW);

                resetDriveEncoders();

                moveForward();

                state = MOVE_30;
            }

            break;

        // ----------------------------------
        case MOVE_30:

            if(getDriveDistanceCM() >= DIST_3)
            {
                driveMode = REAR_ONLY;
                stopDrive();

                digitalWrite(RELAY_PIN, LOW);

                bldc.writeMicroseconds(BLDC_REVERSE_SPEED);

                state = WAIT_BLDC_DOWN;
            }

            break;

        // ----------------------------------
        case WAIT_BLDC_DOWN:

            if(getBLDCDistanceCM() <= BLDC_DOWN_POS)
            {
                bldc.writeMicroseconds(BLDC_STOP_SPEED);

                resetDriveEncoders();

                moveForward();

                state = MOVE_20_FINAL;
            }

            break;

        // ----------------------------------
        case MOVE_20_FINAL:

            if(getDriveDistanceCM() >= DIST_4)
            { 
                driveMode = FRONT_ONLY;
                stopAll();

                state = FINISHED;
            }

            break;

        // ----------------------------------
        case FINISHED:

            stopAll();

            break;
    }
}