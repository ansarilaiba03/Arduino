
// Motor 1 pins
#define M1_STEP 17
#define M1_DIR  16

// Motor 2 pins (left)
#define M2_STEP 19
#define M2_DIR  18

// Timing variables
unsigned long previousMillis = 0;
const unsigned long stepInterval = 1;  // milliseconds between steps

// Step states
bool m1StepState = false;
bool m2StepState = false;

void setup() {
  pinMode(M1_STEP, OUTPUT);
  pinMode(M1_DIR, OUTPUT);
  pinMode(M2_STEP, OUTPUT);
  pinMode(M2_DIR, OUTPUT);

  // Both motors forward
  digitalWrite(M1_DIR, HIGH);
  digitalWrite(M2_DIR, HIGH);

  digitalWrite(M1_STEP, LOW);
  digitalWrite(M2_STEP, LOW);
}
cv
void loop() {
  unsigned long currentMillis = millis();

  // Check if it's time for the next step
  if (currentMillis - previousMillis >= stepInterval) {
    previousMillis = currentMillis;

    // Toggle motor 1 step pin
    m1StepState = !m1StepState;
    digitalWrite(M1_STEP, m1StepState);

    // Toggle motor 2 step pin
    m2StepState = !m2StepState;
    digitalWrite(M2_STEP, m2StepState);
  }
}


