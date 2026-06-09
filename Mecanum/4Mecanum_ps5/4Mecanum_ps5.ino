#include <ps5Controller.h>

#define STOP           0
#define FORWARD        1 
#define BACKWARD       2
#define LEFT           3
#define RIGHT          4
#define FORWARD_LEFT   5
#define FORWARD_RIGHT  6
#define BACKWARD_LEFT  7
#define BACKWARD_RIGHT 8
#define TURN_LEFT      9
#define TURN_RIGHT     10

#define BACK_RIGHT_MOTOR  0
#define BACK_LEFT_MOTOR   1
#define FRONT_RIGHT_MOTOR 2
#define FRONT_LEFT_MOTOR  3

struct MOTOR_PINS {
  int pinDIR;
  int pinPWM;
  int pwmChannel; // Internal ESP32 PWM channel (NOT physical pin)
};

// List of all motors with their pins and PWM channels
std::vector<MOTOR_PINS> motorPins = {
  {16, 17, 0}; //BACK_RIGHT_MOTOR
  {18, 19, 1}; //BACK_LEFT_MOTOR
  {20, 21, 2}; //FRONT_RIGHT_MOTOR
  {22, 23, 3}; //FRONT_LEFT_MOTOR
};

#define MAX_MOTOR_SPEED 200

const int PWMFreq = 1000;
const int PWMResolution =8;

#define SIGNAL_TIMEOUT 1000 // Stop robot if no signal received for 1000 ms

unsigned long lastRecvTime = 0; // Stores last time controller sent data

// This function runs automatically whenever controller sends data
void notify(){
  // Create a string to print joystick values on Serial Monitor
  String inputData;
  inputData = inputData + "values " + ps5.RStickX() + "  " + ps5.RStickY() + "  " + ps5.LStickX();

  Serial.println(inputData); // Print joystick values for debugging

  // Check joystick positions and decide movement

  if ( ps5.RStickX() < -75 && ps5.RStickY() > 75 ) 
  processCarMovement(FORWARD_LEFT);

  else if ( ps5.RStickX() > 75 && ps5.RStickY() > 75) 
  processCarMovement(FORWARD_RIGHT);

  else if ( ps5.RStickX() < - 75 && ps5.RStickY < -75 )
  processCarMovement(BACKWARD_LEFT);

  else if ( ps5.RStickX() > 75 && ps5.RStickY < -75 )
  processCarMovement(BACKWARD_RIGHT);

  else if ( ps5.LStickX() > 75 )
  processCarMovement(TURN_RIGHT);

  else if ( ps5.LStickX() < -75 )
  processCarMovement(TURN_LEFT);

  else if ( ps5.RStickY() > 75 )
  processCarMovement(FORWARD);

  else if ( ps5.RStickY() < -75 )
  processCarMovement(BACKWARD);

  else if ( ps5.RStickX() > 75 )
  processCarMovement(RIGHT);

  else if ( ps5.RStickX < -75 )
  processCarMovement(LEFT);

  else
  processCarMovement(STOP);

  // Update last received time (used for safety timeout)
  lastReacTime = millis();
}

void onConnect(){
  Serial.println("Connected");
}

void onDisconnect(){
  processCarMovement(STOP);
  Serial.println("Disconnected");
}

void processCarMovement(int inputValue){
  switch(inputValue){

  case FORWARD:
    rotateMotor(FRONT_RIGHT_MOTOR, MAX_MOTOR_SPEED);
    rotateMotor(BACK_RIGHT_MOTOR, MAX_MOTOR_SPEED);
    rotateMotor(FRONT_LEFT_MOTOR, MAX_MOTOR_SPEED);
    rotateMotor(BACK_LEFT_MOTOR, MAX_MOTOR_SPEED);
    break;

  case BACKWARD:
    rotateMotor(FRONT_RIGHT_MOTOR, -MAX_MOTOR_SPEED);
    rotateMotor(BACK_RIGHT_MOTOR, -MAX_MOTOR_SPEED);
    rotateMotor(FRONT_LEFT_MOTOR, -MAX_MOTOR_SPEED);
    rotateMotor(BACK_LEFT_MOTOR, -MAX_MOTOR_SPEED);
    break;

  case LEFT:
    rotateMotor(FRONT_RIGHT_MOTOR, MAX_MOTOR_SPEED);
    rotateMotor(BACK_RIGHT_MOTOR, -MAX_MOTOR_SPEED);
    rotateMotor(FRONT_LEFT_MOTOR, -MAX_MOTOR_SPEED);
    rotateMotor(BACK_LEFT_MOTOR, MAX_MOTOR_SPEED);
    break;

  case RIGHT:

    rotateMotor(FRONT_RIGHT_MOTOR, -MAX_MOTOR_SPEED);
    rotateMotor(BACK_RIGHT_MOTOR, MAX_MOTOR_SPEED);
    rotateMotor(FRONT_LEFT_MOTOR, MAX_MOTOR_SPEED);
    rotateMotor(BACK_LEFT_MOTOR, -MAX_MOTOR_SPEED);
    break;

  case FORWARD_LEFT:
    rotateMotor(FRONT_RIGHT_MOTOR, MAX_MOTOR_SPEED);
    rotateMotor(BACK_RIGHT_MOTOR, STOP);
    rotateMotor(FRONT_LEFT_MOTOR, STOP);
    rotateMotor(BACK_LEFT_MOTOR, MAX_MOTOR_SPEED);  
    break;
  
  case FORWARD_RIGHT:
    rotateMotor(FRONT_RIGHT_MOTOR, STOP);
    rotateMotor(BACK_RIGHT_MOTOR, MAX_MOTOR_SPEED);
    rotateMotor(FRONT_LEFT_MOTOR, MAX_MOTOR_SPEED);
    rotateMotor(BACK_LEFT_MOTOR, STOP);  
    break;
  
  case BACKWARD_LEFT:
    rotateMotor(FRONT_RIGHT_MOTOR, STOP);
    rotateMotor(BACK_RIGHT_MOTOR, -MAX_MOTOR_SPEED);
    rotateMotor(FRONT_LEFT_MOTOR, -MAX_MOTOR_SPEED);
    rotateMotor(BACK_LEFT_MOTOR, STOP);   
    break;

  case BACKWARD_RIGHT:
    rotateMotor(FRONT_RIGHT_MOTOR, -MAX_MOTOR_SPEED);
    rotateMotor(BACK_RIGHT_MOTOR, STOP);
    rotateMotor(FRONT_LEFT_MOTOR, STOP);
    rotateMotor(BACK_LEFT_MOTOR, -MAX_MOTOR_SPEED);   
    break;
  
  case TURN_LEFT:
    rotateMotor(FRONT_RIGHT_MOTOR, MAX_MOTOR_SPEED);
    rotateMotor(BACK_RIGHT_MOTOR, MAX_MOTOR_SPEED);
    rotateMotor(FRONT_LEFT_MOTOR, -MAX_MOTOR_SPEED);
    rotateMotor(BACK_LEFT_MOTOR, -MAX_MOTOR_SPEED);  
    break;
  
  case TURN_RIGHT:
    rotateMotor(FRONT_RIGHT_MOTOR, -MAX_MOTOR_SPEED);
    rotateMotor(BACK_RIGHT_MOTOR, -MAX_MOTOR_SPEED);
    rotateMotor(FRONT_LEFT_MOTOR, MAX_MOTOR_SPEED);
    rotateMotor(BACK_LEFT_MOTOR, MAX_MOTOR_SPEED);   
    break;  

  case STOP:

  default:

  rotateMotor(FRONT_RIGHT_MOTOR, STOP);
  rotateMotor(BACK_RIGHT_MOTOR, STOP);
  rotateMotor(FRONT_LEFT_MOTOR, STOP);
  rotateMotor(BACK_LEFT_MOTOR, STOP);
   break;
  }
}

void rotateMotor(int motorNumber, int motorSpeed) {
  if (motorSpeed >= 0) {
    digitalWrite(motorPins[motorNumber].pinDIR, HIGH);
  }

  else  {
    digitalWrite(motorPins[motorNumber].pinDIR, LOW);
    motorSpeed = -motorSpeed; // Convert to positive for PWM
  }

  // Send speed signal using PWM channel
  ledcWrite(motorPins[motorNumber].pwmChannel, motorSpeed);
}

// Setup motor pins and PWM channels
void setUpPinModes() {
  for (int i = 0; i < motorPins.size(); i++) {
    pinMode(motorPins[i].pinDIR, OUTPUT);
    ledcSetup(motorPins[i].pwmChannel, PWMFreq, PWMResolution);
    ledcAttachPin(motorPins[i].pinPWM, motorPins[i].pwmChannel);
    rotateMotor(i, STOP);
  }
}

void setup() {
  setUpPinModes();
  Serial.begin(9600);
  ps5.attach(notify);
  ps5.attachOnConnect(onConnect);
  ps5.attachOnDisconnect(onDisConnect);
  ps5.begin("7C:66:EF:78:76:F0"); // MAC  
  Serial.println("Ready.");
}

void loop() {
  unsigned long now = millis();

  // If controller disconnected, stop robot
  if ( now - lastRecvTime > SIGNAL_TIMEOUT ) {
    processCarMovement(STOP);
  }
}