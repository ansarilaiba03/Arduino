#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// ---------------- ENCODER PINS ----------------
const int encoderLeftA = 2;  // white
const int encoderLeftB = 28; // green
const int encoderRightA = 19; // green
const int encoderRightB = 34;// white

// ---------------- PROXIMITY SENSOR ----------------
const int proxPin1 = 38;    
const int proxPin2 = 42;   // side sensor 
bool bypassingObstacle = false;  
bool bypassing = false;
bool movingSide = false;
float lastTurnYaw = 0;

// ---------------- MOTOR PINS ----------------
const int motorLeftPWM = 6;
const int motorLeftDIR = 30;
const int motorRightPWM = 8;
const int motorRightDIR = 32;

// ---------------- ROBOT CONSTANTS ----------------
const float pulsesPerRevLeft = 400.0;
const float pulsesPerRevRight = 400.0;
const float wheelDiameter = 152.0;   // mm
const float wheelCircumference = PI * wheelDiameter;
const float wheelBase = 455.0;       // mm

// ---------------- ENCODER COUNTS ----------------
volatile long leftPulses = 0;
volatile long rightPulses = 0;
long prevLeftPulses = 0;
long prevRightPulses = 0;

// ---------------- POSITION VARIABLES ----------------
float x = 0.0;
float y = 0.0;
float theta = 0.0; // degrees
float yawReference = 0;
static double current_wl = 0;  
static double current_wr = 0;   

// ---------------- PID Constants ----------------
double Kp = 2.0;
double Ki = 0.0;
double Kd = 0.1;

// --- Separate PID constants for Yaw Control ---
double Kp_yaw = 1.1;
double Ki_yaw = 0.0;
double Kd_yaw = 1.0;

// ---------------- PID Variables ----------------
double setpoint = 0;
double input = 0;
double output = 0;
double error, lastError = 0, integral = 0, derivative = 0; 
unsigned long lastTime = 0;

// ---------------- IMU ----------------
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// ---------------- TARGET COORDINATES ----------------
float xTarget = 2000.0;   // mm → move forward 500 mm
float turnAngle = 90.0;  // degrees → turn right 90°
float yTarget = 2000.0;   // mm → then move forward 300 mm

bool phase1_done = false;
bool turn_done = false;
bool phase2_done = false;


// ---------------- INTERRUPTS ----------------
void leftEncoderISR() {
  int A = digitalRead(encoderLeftA);
  int B = digitalRead(encoderLeftB);
  if (A == B) leftPulses++;
  else leftPulses--;
}

void rightEncoderISR() {
  int A = digitalRead(encoderRightA);
  int B = digitalRead(encoderRightB);
  if (A == B) rightPulses++;
  else rightPulses--;
}

// ---------------- HELPER FUNCTIONS ----------------
float getYaw() {
  imu::Quaternion quat = bno.getQuat();
  float qw = quat.w(), qx = quat.x(), qy = quat.y(), qz = quat.z();
  float yawRad = atan2(2.0 * (qw * qz + qx * qy),
                       1.0 - 2.0 * (qy * qy + qz * qz));
  float yawDeg = yawRad * 180.0 / PI;
  if (yawDeg < 0) yawDeg += 360.0;
   // clockwise
  yawDeg = 360.0 - yawDeg;
  if (yawDeg >= 360) yawDeg -= 360;
  return yawDeg;
}

// --- Distance PID ---
double computeDistancePID(double target, double current) {
  unsigned long now = millis();
  double dt = (now - lastTime) / 1000.0;
  if (dt <= 0) dt = 0.001;

  double error = target - current;
  integral += error * dt;
  derivative = (error - lastError) / dt;

  double output = Kp * error + Ki * integral + Kd * derivative;
  lastError = error;
  lastTime = now;
  output = constrain(output, -50, 50);
  return output;
}

// --- Yaw PID ---
double computeYawPID(double setpoint, double input) {
  static double lastYawError = 0, yawIntegral = 0;
  static unsigned long lastYawTime = 0;

  unsigned long now = millis();
  double dt = (now - lastYawTime) / 1000.0;
  if (dt <= 0) dt = 0.001;

  double error = setpoint - input;   
  if (error > 180) error -= 360;
  if (error < -180) error += 360;

  yawIntegral += error * dt;
  double derivative = (error - lastYawError) / dt;

  double output = Kp_yaw * error + Ki_yaw * yawIntegral + Kd_yaw * derivative;
  output = constrain(output, -50, 50);
  lastYawError = error;
  lastYawTime = now;

  return output;
}
// // ---------------- VIRTUAL WALL + PLEDGE HELPERS ----------------

// // bool hitBoundary() {
// //    return (x <= -2000 || x >= 2000 || y <= -2000 || y >= 2000);
// // }

// bool boxAhead() {
//    return digitalRead(proxPin) == LOW;  
// }

// bool facingGoal() {
//    float err = getYaw() - goalHeading;
//    if (err > 180) err -= 360;
//    if (err < -180) err += 360;
//    return abs(err) < 10;
// }

bool frontObstacle() {
  return digitalRead(proxPin1) == LOW;
}

bool sideObstacle() {
  return digitalRead(proxPin2) == LOW;
}
// --- Drive function with yaw correction ---
void drive(double forwardPID, double yawCorrection) {
  static double current_wl = 0;  
  static double current_wr = 0;   
  double leftPWM = constrain(forwardPID - (yawCorrection*0), -50, 50);
  double rightPWM = constrain(forwardPID + (yawCorrection*0), -50, 50);
  current_wl += (leftPWM - current_wl) / 1;
  current_wr += (rightPWM - current_wr) /1;

  if (current_wl >= 0) {
    digitalWrite(motorLeftDIR, LOW);
    analogWrite(motorLeftPWM, current_wl);
  } else {
    digitalWrite(motorLeftDIR, HIGH);
    analogWrite(motorLeftPWM, -current_wl);
  }

  if (current_wr >= 0) {
    digitalWrite(motorRightDIR, LOW);
    analogWrite(motorRightPWM, current_wr);
  } else {
    digitalWrite(motorRightDIR, HIGH);
    analogWrite(motorRightPWM, -current_wr);
  }
}

void moveForwardDistance(float distanceMM) {
    leftPulses = 0;
    rightPulses = 0;

    long targetPulses = (distanceMM / wheelCircumference) * pulsesPerRevLeft;

    while (true) {
        long avgPulses = (abs(leftPulses) + abs(rightPulses)) / 2;

        if (avgPulses >= targetPulses) break;

        // Move straight
        digitalWrite(motorLeftDIR, LOW);
        digitalWrite(motorRightDIR, LOW);

        analogWrite(motorLeftPWM, 50);
        analogWrite(motorRightPWM, 50);
    }

    analogWrite(motorLeftPWM, 0);
    analogWrite(motorRightPWM, 0);
    delay(200);
}


void performRightBypass() {

    float yawStart = getYaw();
    float yawTarget = yawStart - 90;
    if (yawTarget >= 360) yawTarget -= 360;

    Serial.println("Turning RIGHT 90°...");
    rotateToYaw(yawTarget);

    Serial.println("Moving forward 400mm...");
    moveForwardDistance(400);

    float yawTarget2 = yawStart;  // left 90° back to original heading
    Serial.println("Turning LEFT 90°...");
    rotateToYaw(yawTarget2);
}

void performLeftBypass() {

    float yawStart = getYaw();
    float yawTarget = yawStart + 90;
    if (yawTarget < 0) yawTarget += 360;

    Serial.println("Turning LEFT 90°...");
    rotateToYaw(yawTarget);

    Serial.println("Moving forward 400mm...");
    moveForwardDistance(400);

    float yawTarget2 = yawStart;  // right 90° back to original heading
    Serial.println("Turning RIGHT 90°...");
    rotateToYaw(yawTarget2);
}

void rotateToYaw(float targetYaw) {

    while (true) {
        float currentYaw = getYaw();
        float error = currentYaw - targetYaw;

        if (error > 180) error -= 360;
        if (error < -180) error += 360;

        int pwm = constrain(abs(error * 2), 30, 90);

        if (error > 0) {
            digitalWrite(motorLeftDIR, LOW);
            digitalWrite(motorRightDIR, HIGH);
        } else {
            digitalWrite(motorLeftDIR, HIGH);
            digitalWrite(motorRightDIR, LOW);
        }

        analogWrite(motorLeftPWM, pwm);
        analogWrite(motorRightPWM, pwm);

        if (abs(error) < 2) break;
    }

    analogWrite(motorLeftPWM, 0);
    analogWrite(motorRightPWM, 0);
    delay(200);
}


// --- Smooth Stop ---
void gradualStop() {
  // for (int pwm = 100; pwm >= 0; pwm -= 2) {
  //   analogWrite(motorLeftPWM, pwm);
  //   analogWrite(motorRightPWM, pwm);
  //   delay(20);
  // }
  // analogWrite(motorLeftPWM, 0);
  // analogWrite(motorRightPWM, 0);
}

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(9600);
  Wire.begin();

  if (!bno.begin()) {
    Serial.println("BNO055 not detected!");
    while (1);
  }
  bno.setExtCrystalUse(true);
  delay(1000);

  pinMode(encoderLeftA, INPUT_PULLUP);
  pinMode(encoderLeftB, INPUT_PULLUP);
  pinMode(encoderRightA, INPUT_PULLUP);
  pinMode(encoderRightB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoderLeftA), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderRightA), rightEncoderISR, RISING);

  pinMode(motorLeftPWM, OUTPUT);
  pinMode(motorLeftDIR, OUTPUT);
  pinMode(motorRightPWM, OUTPUT);
  pinMode(motorRightDIR, OUTPUT);
  pinMode(proxPin1, INPUT);
  pinMode(proxPin2, INPUT);

  Serial.println("Starting Auto Navigation...");
  lastTime = millis();
  yawReference = getYaw();
  Serial.print("Yaw reference = ");
  Serial.println(yawReference);

}

// ---------------- LOOP ----------------
void loop() {
// -------------------------
// OBSTACLE DETECTION + BYPASS LOGIC
// -------------------------
if (!bypassing && frontObstacle()) {
    // Obstacle detected → take right turn
    gradualStop();
    rotateToYaw((theta, 90));  // right turn
    lastTurnYaw = getYaw();
    bypassing = true;
    movingSide = true;
}

else if (movingSide && sideObstacle()) {
    // Move along the bypass path until side sensor clears
    drive(50, 0); // drive forward with no yaw correction
}

else if (movingSide && !sideObstacle()) {
    gradualStop();
    // After bypass, align to yaw = 0 (North)
    rotateToYaw(0);
    bypassing = false;
    movingSide = false;
}

  long currentLeft = leftPulses;
  long currentRight = rightPulses;
  long deltaLeft = currentLeft - prevLeftPulses;
  long deltaRight = currentRight - prevRightPulses;
  prevLeftPulses = currentLeft;
  prevRightPulses = currentRight;

  float dLeft = (deltaLeft / pulsesPerRevLeft) * wheelCircumference;
  float dRight = (deltaRight / pulsesPerRevRight) * wheelCircumference;
  float dCenter = (dRight + dLeft) / 2.0;

  theta = getYaw();
  float thetaRad = radians(theta);
  x += dCenter * cos(thetaRad);
  y += dCenter * sin(thetaRad);
  input += dCenter;

  // --- PHASE 1: Move forward to x  ---
  if (!phase1_done) {
    double yaw = getYaw();
    double yawCorrection = computeYawPID(yawReference, yaw);
    double distanceError = xTarget - x;
    double pidOut = computeDistancePID(xTarget, x);

    if (distanceError < 150 && distanceError > 0)
      pidOut *= distanceError / 150.0;

    drive(pidOut, yawCorrection);

    if (fabs(distanceError) < 10) {
      gradualStop();
      Serial.println("Reached X target!");
      phase1_done = true;
      input = 0;
      integral = 0;
      lastError = 0;
      delay(1000);
    }
  }

  // --- PHASE 2: Turn right to yaw = 90° using old PID logic ---
else if (!turn_done) {

    float targetYaw = turnAngle; 
    float currentYaw = getYaw();

    float error = currentYaw - targetYaw;
    if (error > 180) error -= 360;
    if (error < -180) error += 360;

    // PID for yaw
    double pidOut = computeYawPID(targetYaw, currentYaw);

    int pwm = constrain(abs(pidOut), 25, 80);  // min speed to rotate

    // Rotate robot
    if (pidOut > 0) {
        digitalWrite(motorLeftDIR, LOW);
        digitalWrite(motorRightDIR, HIGH);
    } else {
        digitalWrite(motorLeftDIR, HIGH);
        digitalWrite(motorRightDIR, LOW);
    }

    analogWrite(motorLeftPWM, pwm);
    analogWrite(motorRightPWM, pwm);

    // Turn complete?
    if (abs(error) < 2) {
        analogWrite(motorLeftPWM, 0);
        analogWrite(motorRightPWM, 0);

        Serial.println("Rotation Done!");
        turn_done = true;

        // Reset PID accumulators
        integral = 0;
        lastError = 0;
        input = 0;

        delay(500);
    }

    return;   // IMPORTANT — stops loop from continuing
}


  // --- PHASE 3: Move forward to y = 300 at yaw = 90° ---
  else if (!phase2_done) {
    //input = -input;  
    double yaw = getYaw();
    double yawCorrection = computeYawPID(90, yaw);
    double distanceError = yTarget - y  ;
    double pidOut = computeDistancePID(yTarget, y);
    //pidOut = -pidOut;  // Reverse direction so forward = right

    if (abs(distanceError) < 100)
      pidOut *= abs(distanceError) / 100.0;

    drive(pidOut, yawCorrection);

   if (abs(y - yTarget) < 30) {
    gradualStop();
    analogWrite(motorLeftPWM, 0);
    analogWrite(motorRightPWM, 0);

    current_wl = 0;  // <-- reset drive internal state
    current_wr = 0;

    Serial.println("Y reached");
    phase2_done = true;
    delay(1000);
    integral = 0;
    lastError = 0;
    lastTime = millis();

    // Force motors off
    analogWrite(motorLeftPWM, 0);
    analogWrite(motorRightPWM, 0);

    // small delay to stabilize
    delay(100);

}

    // Serial.print("Input = ");
    // Serial.println(input);

  }
  // --- PHASE 4: Final Correction Phase (Check Phase1 and Phase3) ---
else if (phase1_done && phase2_done) {

    float yawNow = getYaw();

    bool xCorrect = abs(x - xTarget) < 20;
    bool yCorrect = abs(y - yTarget) < 20;
    bool yawCorrect = abs(yawNow - 90) < 5;

// --- NEW: always print debug ---
     Serial.print("Phase4: X = "); Serial.print(x, 1);
    Serial.print("  Y = "); Serial.print(y, 1);
    Serial.print("  Theta = "); Serial.print(yawNow, 1);
    Serial.print("  xCorrect = "); Serial.print(xCorrect);
    Serial.print("  yCorrect = "); Serial.print(yCorrect);
    Serial.print("  yawCorrect = "); Serial.println(yawCorrect);

 // --- stop robot if everything is correct ---
if (xCorrect && yCorrect && yawCorrect) {
    analogWrite(motorLeftPWM, 0);
    analogWrite(motorRightPWM, 0);
    current_wl = 0;  // reset internal drive state
    current_wr = 0;

    Serial.println("ALL PHASES COMPLETE! Robot fully stopped.");
    return;  // stop PID corrections
}

    if (!xCorrect) {
        //Serial.println("Fixing X alignment...");
        double yawCorrection = computeYawPID(90, yawNow);
        double pidOut = computeDistancePID(xTarget, x);
        if (abs(x - xTarget) < 100) {   // DEAD ZONE
       analogWrite(motorLeftPWM, 0);
       analogWrite(motorRightPWM, 0);
        current_wl = 0;
        current_wr = 0;
        Serial.println("X close enough. Stopped X correction.");
        xCorrect = true;
      // return;
}
        pidOut = constrain(pidOut, -70, 70);
        drive(pidOut, yawCorrection);
        //return;
    }

    if (!yCorrect) {
        //Serial.println("Fixing Y alignment...");
        double yawCorrection = computeYawPID(90, yawNow);
        double pidOut = computeDistancePID(yTarget, y);
        if (abs(y - yTarget)< 100) {   // DEAD ZONE
        analogWrite(motorLeftPWM, 0);
        analogWrite(motorRightPWM, 0);
        current_wl = 0;
        current_wr = 0;
        Serial.println("Y close enough. Stopped Y correction.");
        yCorrect = true;
       //return;
        }
        pidOut = constrain(pidOut, -70, 70);
        drive(pidOut, yawCorrection);
        //return;
    }

    if (!yawCorrect) {
        //Serial.println("Fixing final yaw...");
        if (abs(yawNow - 90) < 2) {
    analogWrite(motorLeftPWM, 0);
    analogWrite(motorRightPWM, 0);
    current_wl = 0;
    current_wr = 0;
    Serial.println("Yaw close enough. Stopped rotation.");
    yawCorrect = true;
    //return;
}
        rotateToYaw(90);
       // return;
   }

    // ALL GOOD
    Serial.println("ALL PHASES CORRECT! ROBOT READY.");
    analogWrite(motorLeftPWM, 0);
    analogWrite(motorRightPWM, 0);
    integral = 0;
    lastError = 0;
    lastTime = millis();
    return;
}


  // --- DONE ---
  else {
    analogWrite(motorLeftPWM, 0);
    analogWrite(motorRightPWM, 0);
  }

 
 // -------------------------
// ALWAYS SHOW DEBUG DATA
// -------------------------
float yawNow = getYaw();

// Distance error depending on phase
float distanceErrorDebug = 0;
if (!phase1_done) distanceErrorDebug = xTarget - input;
else if (!turn_done) distanceErrorDebug = 0;
else if (!phase2_done) distanceErrorDebug = yTarget - input;

// Yaw error depending on phase  
float yawErrorDebug = 0;
if (!phase1_done) yawErrorDebug = yawNow - 0;
else if (!turn_done) yawErrorDebug = yawNow - turnAngle;
else if (!phase2_done) yawErrorDebug = yawNow - 90;

// Keep yaw error in -180..180
if (yawErrorDebug > 180) yawErrorDebug -= 360;
if (yawErrorDebug < -180) yawErrorDebug += 360;

Serial.print("X = ");     Serial.print(x, 1);
Serial.print("  Y = ");   Serial.print(y, 1);
Serial.print("  Theta = "); Serial.print(yawNow, 1);

Serial.print("  DistErr = ");
Serial.print(distanceErrorDebug);

Serial.print("  YawErr = ");
Serial.println(yawErrorDebug);


  // --- PID tuning via Serial ---
  if (Serial.available() > 0) {
    String inputStr = Serial.readStringUntil('\n');
    inputStr.trim();
    if (inputStr.startsWith("kp")) {
      Kp = inputStr.substring(2).toFloat();
      Serial.print("Kp set to: "); Serial.println(Kp);
    } else if (inputStr.startsWith("ki")) {
      Ki = inputStr.substring(2).toFloat();
      Serial.print("Ki set to: "); Serial.println(Ki);
    } else if (inputStr.startsWith("kd")) {
      Kd = inputStr.substring(2).toFloat();
      Serial.print("Kd set to: "); Serial.println(Kd);
    }
  }

  delay(100);
}