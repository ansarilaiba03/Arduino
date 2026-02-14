// ------------------- PID Constants -------------------
const double Kp = 2.0;   // Proportional gain
const double Ki = 5.0;   // Integral gain
const double Kd = 1.0;   // Derivative gain

// ------------------- Encoder & Motor Setup -------------------
volatile long pulseCount = 0;

const int motorPWM = 9;   // PWM pin to driver
const int motorDIR = 8;   // Direction pin to driver
const int encoderA = 2;   // Encoder Channel A (interrupt pin)
const int encoderB = 3;   // Encoder Channel B (direction check)

// Replace this with your encoder's actual pulses per revolution
//const int PULSES_PER_REV = 135;  

// ------------------- PID Variables -------------------
double setpoint;      // Target position (pulses)
double input;         // Current position (pulses)
double output;        // PID controller output (PWM value)
double error, lastError;
double integral, derivative;
unsigned long lastTime;

// ------------------- Encoder ISR -------------------
void encoderISR() {
  // Quadrature decoding: check channel B for direction
  if (digitalRead(encoderB) == HIGH) {
    pulseCount++;
  } else {
    pulseCount--;
  }
}

// ------------------- Setup -------------------
void setup() {
  Serial.begin(9600);

  // Encoder setup
  pinMode(encoderA, INPUT_PULLUP);
  pinMode(encoderB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderA), encoderISR, RISING);

  // Motor pins
  pinMode(motorPWM, OUTPUT);
  pinMode(motorDIR, OUTPUT);

  // Target = 90 degrees
  setpoint = 500; //PULSES_PER_REV / 4;   // 90° = 1/4 of full rotation

  // Initialize PID vars
  lastTime = millis();
  lastError = 0;
  integral = 0;
}

// ------------------- Loop -------------------
void loop() {
  input = pulseCount;   // Feedback = current encoder position

  // ----------- Manual PID Calculation -----------
  unsigned long now = millis();
  double dt = (now - lastTime) / 1000.0; // seconds
  if (dt <= 0) dt = 0.001; // avoid divide by zero

  error = setpoint - input;

  integral += error * dt;                 // Integral term
  //derivative = (error - lastError) / dt;  // Derivative term

  output = Kp * error + Ki * integral ;//+ Kd * derivative;

  // Save for next loop
  lastError = error;
  lastTime = now;

  // Limit output to -255..255
  if (output > 255) output = 255;
  if (output < -255) output = -255;

  // ----------- Apply PID output to motor -----------
  if (output > 0) {
    digitalWrite(motorDIR, HIGH);     // CW
    analogWrite(motorPWM, output);
  } else {
    digitalWrite(motorDIR, LOW);      // CCW
    analogWrite(motorPWM, -output);
  }

  Serial.print("Current Count: ");
  Serial.println(input);

  // ----------- Stop at target position -----------
  if (abs(setpoint - input) < 2) {
    analogWrite(motorPWM, 0);         // Stop motor
    Serial.println("Reached 500 counts");
    while (1);  // Stop program here
  }
}
