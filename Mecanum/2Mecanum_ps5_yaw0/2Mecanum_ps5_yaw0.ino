#include <Arduino.h>
#include <ps5Controller.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <math.h>

// ------------------- IMU -------------------
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// ------------------- PID Constants -------------------
double Kp = 6.0;
double Ki = 0.2;
double Kd = 0.1;

double setpoint = 0.0;
double error, lastError = 0;
double integral = 0, derivative;
double output;
unsigned long lastTime;

// ------------------- Motor Pins -------------------
// Left Motor
const int LEFT_PWM_PIN = 18;
const int LEFT_DIR_PIN = 19;
// Right Motor
const int RIGHT_PWM_PIN = 16;
const int RIGHT_DIR_PIN = 17;

// Robot geometry
const int b = 325; // mm
const int r = 76;  // mm

// PS5 Joystick
int lx, ly, rx, ry;
int buff = 10;
int deadzone = 0;

// Wheel speeds
double targetwl = 0, targetwr = 0;

// ------------------- [NEW] Serial Input Buffer -------------------
String inputString = "";
bool stringComplete = false;

// ------------------- Functions -------------------
double readYaw() {
    imu::Quaternion q = bno.getQuat();
    double yaw = atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                       1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
    yaw = yaw * 180.0 / M_PI;
    if (yaw < 0) yaw += 360.0;
    if (yaw > 180) yaw -= 360.0;
    return yaw;
}

double computePID(double currentYaw) {
    unsigned long now = millis();
    double dt = (now - lastTime) / 1000.0;
    if (dt <= 0) dt = 0.001;

    error = currentYaw - setpoint;
    integral += error * dt;
    derivative = (error - lastError) / dt;

    output = Kp * error + Ki * integral + Kd * derivative;

    lastError = error;
    lastTime = now;

    return output;
}

void compute2Wheel(int lyInput, int rxInput, double pidCorrection) {
    float V = lyInput;
    float rX = rxInput * 2;

    if (abs(lyInput) > buff) {
        targetwr = ((V * 200 + (rX * b / 2)) / r) - pidCorrection;
        targetwl = ((V * 200 - (rX * b / 2)) / r) + pidCorrection;
    } else if (abs(pidCorrection) > 2.0) {
        targetwr = pidCorrection;
        targetwl = -pidCorrection;
    } else {
        targetwr = 0;
        targetwl = 0;
    }

    float speedFactor = 0.5;
    targetwr = constrain(targetwr * speedFactor, -255, 255);
    targetwl = constrain(targetwl * speedFactor, -255, 255);
}

void applySpeed() {
    if (targetwl >= 0) {
        digitalWrite(LEFT_DIR_PIN, LOW);
        analogWrite(LEFT_PWM_PIN, targetwl);
    } else {
        digitalWrite(LEFT_DIR_PIN, HIGH);
        analogWrite(LEFT_PWM_PIN, -targetwl);
    }

    if (targetwr >= 0) {
        digitalWrite(RIGHT_DIR_PIN, LOW);
        analogWrite(RIGHT_PWM_PIN, targetwr);
    } else {
        digitalWrite(RIGHT_DIR_PIN, HIGH);
        analogWrite(RIGHT_PWM_PIN, -targetwr);
    }
}

// ------------------- [NEW] Read Serial PID Input -------------------
void readSerialInput() {
    if (stringComplete) {
        inputString.trim();  // Remove leading/trailing spaces

        if (inputString.startsWith("kp=")) {
            Kp = inputString.substring(3).toFloat();
            Serial.print("Kp updated to: "); Serial.println(Kp);
        } else if (inputString.startsWith("ki=")) {
            Ki = inputString.substring(3).toFloat();
            Serial.print("Ki updated to: "); Serial.println(Ki);
        } else if (inputString.startsWith("kd=")) {
            Kd = inputString.substring(3).toFloat();
            Serial.print("Kd updated to: "); Serial.println(Kd);
        } else {
            Serial.println("Unknown command.");
        }

        inputString = "";
        stringComplete = false;
    }
}

// ------------------- [NEW] Serial Event Handler -------------------
void serialEvent() {
    while (Serial.available()) {
        char inChar = (char)Serial.read();
        if (inChar == '\n') {
            stringComplete = true;
        } else {
            inputString += inChar;
        }
    }
}

// ------------------- Setup -------------------
void setup() {
    Serial.begin(9600);

    if (!bno.begin()) {
        Serial.println("BNO055 not detected!");
        while (1);
    }
    delay(1000);
    bno.setExtCrystalUse(true);

    pinMode(LEFT_PWM_PIN, OUTPUT);
    pinMode(RIGHT_PWM_PIN, OUTPUT);
    pinMode(LEFT_DIR_PIN, OUTPUT);
    pinMode(RIGHT_DIR_PIN, OUTPUT);

    lastTime = millis();

    Serial.println("Connecting to PS5...");
    ps5.begin("7c:66:ef:78:76:f0");
    while (!ps5.isConnected()) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nPS5 connected!");

    Serial.println("Send PID values like: kp=4.5 or ki=0.01");
}

// ------------------- Main Loop -------------------
void loop() {
    readSerialInput();  // [NEW] Check for incoming serial commands

    if (ps5.isConnected()) {
        lx = ps5.LStickX();
        ly = ps5.LStickY();
        rx = -ps5.RStickX();
        ry = ps5.RStickY();
    }

    if (abs(ly) < buff) ly = 0;
    else ly = (ly > 0) ? ly - deadzone : ly + deadzone;

    if (abs(rx) < buff) rx = 0;
    else rx = (rx > 0) ? rx - deadzone : rx + deadzone;

    setpoint = 0.0;
    double currentYaw = readYaw();
    double correction = computePID(currentYaw);
    compute2Wheel(ly, rx, correction);
    applySpeed();

    // Debug
    Serial.print("Yaw: "); Serial.print(currentYaw);
    Serial.print(" | Kp: "); Serial.print(Kp);
    Serial.print(" Ki: "); Serial.print(Ki);
    Serial.print(" Kd: "); Serial.print(Kd);
    Serial.print(" | Corr: "); Serial.println(correction);

    delay(20);
}
