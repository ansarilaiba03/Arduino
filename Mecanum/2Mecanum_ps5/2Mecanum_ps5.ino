/*
connections
DIR1 = 17
PWM1 = 16
DIR2 = 19
PWM2 = 18
GND = GND

IMU
VIN = 5V
GND = GND  
SDA = 20 (MEGA)
SCL = 21 (MEGA)

ENCODER
ALEFT = 2 
BLEFT = 3
ARIGHT = 19
BRIGHT = 18
*/
 
#include <Arduino.h>  // Add this for ESP32 functionality
#include <ps5Controller.h>


// Left Motor
const int LEFT_PWM_CHANNEL = 25;   // PWM channel for left motor
const int LEFT_DIR_PIN = 26;  // Direction pin for left motor


// Right Motor
const int RIGHT_PWM_CHANNEL = 16;  // PWM channel for right motor
const int RIGHT_DIR_PIN = 17;  // Direction pin for right motor
const int b = 222; //325; // 106;  //dist between wheels
const int r = 65; //76;      //wheel radius
int lx, // left-right
 ly, //forward-backward
 rx, //turning left-right
 ry, //up-down (unused)
 tri, cir, squ, cro, l2, r2,l1,r1,left,right,up,down;
int buff = 5;   //ignores small stick movements
double targetwl, //speed for left wheel
targetwr; // speed for right wheel
int deadzone=0; //amt to subtract from stick input for smooth control


//calculate left-right wheels speed using joystick input
void compute2Wheel()
{
  float V = ly;
  float rX = rx*2; // *2 to increase the effect of turning
  targetwr=((ly*200+(rX*b/2))/r);
  targetwl=((ly*200-(rX*b/2))/r); 

  //speed  control
  float speedFactor = 0.1;  // Change this value to control speed
  targetwr = targetwr * speedFactor;
  targetwl = targetwl * speedFactor;
  // -------------------------

  // Limit wheel speeds so they stay within valid motor range
  targetwr = constrain(targetwr, -255, 255);
  targetwl = constrain(targetwl, -200, 200);

  // print wheel speeds
  Serial.print("Right Wheel: ");
  Serial.print(targetwr);
  Serial.print("  Left Wheel: ");
  Serial.println(targetwl); 
}


// sends the calculated speed to motors & sets motor direction depending +ve or -ve speed
void applySpeed()
{
  if (targetwl >= 0) 
  {
  digitalWrite(LEFT_DIR_PIN, LOW); // LOW = forward
  analogWrite(LEFT_PWM_CHANNEL, targetwl);
  } 
  else
  {
  digitalWrite(LEFT_DIR_PIN, HIGH); // HIGH = backward
  analogWrite(LEFT_PWM_CHANNEL, -targetwl);
  }
  if (targetwr >= 0) 
  {
  digitalWrite(RIGHT_DIR_PIN, LOW); 
  analogWrite(RIGHT_PWM_CHANNEL, targetwr);
  } else {
  digitalWrite(RIGHT_DIR_PIN, HIGH);
  analogWrite(RIGHT_PWM_CHANNEL, -targetwr);
  }

}

void setup() {
  Serial.begin(9600);
  // ps5.begin("e8:47:3a:5a:a3:66");
  ps5.begin("7c:66:ef:78:76:f0");

  // pwm
  pinMode(LEFT_PWM_CHANNEL, OUTPUT);
  pinMode(RIGHT_PWM_CHANNEL, OUTPUT);

  // Set direction pins as outputs
  pinMode(LEFT_DIR_PIN, OUTPUT);
  pinMode(RIGHT_DIR_PIN, OUTPUT);
}

void loop() {
  if (ps5.isConnected()) {
    lx = ps5.LStickX();  
    ly = ps5.LStickY();
    rx = -ps5.RStickX();
    ry = ps5.RStickY();
    tri = ps5.Triangle();
    cir = ps5.Circle();
    cro = ps5.Cross();
    squ = ps5.Square();
    l2 = ps5.L2();
    r2 = ps5.R2();
    l1 = ps5.L1();
    r1 = ps5.R1();
    up=ps5.Up();
    down=ps5.Down();
    left=ps5.Left();
    right=ps5.Right();
  }

  if (abs(ly) < buff) ly = 0;
    else ly = (ly > 0) ? ly - deadzone : ly + deadzone;

  if (abs(rx) < buff) rx = 0;
    else rx = (rx > 0) ? rx - deadzone : rx + deadzone;
  compute2Wheel();
  applySpeed();

    Serial.print(lx);
    Serial.print(" ");
    Serial.print(ly);
    Serial.print(" ");
    Serial.print(rx);
    Serial.print(" ");
    Serial.print(ry);
    Serial.print(" ");
    Serial.print(tri);
    Serial.print(" ");
    Serial.print(cir);
    Serial.print(" ");
    Serial.print(cro);
    Serial.print(" ");
    Serial.print(squ);
    Serial.print(" ");
    Serial.print(up);
    Serial.print(" ");
    Serial.print(down);
    Serial.print(" ");
    Serial.print(left);
    Serial.print(" ");
    Serial.print(right);
    Serial.print(" ");
    Serial.print(l1);
    Serial.print(" ");
    Serial.print(r1);
    Serial.print(" ");
    Serial.print(l2);
    Serial.print(" ");
    Serial.println(r2);
    delay(1000);
}