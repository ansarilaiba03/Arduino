// Motor pins
#define STEP_PIN 17
#define DIR_PIN 16

// Timing variables
unsigned long previousMillis = 0;
const unsigned long stepInterval = 2; // milliseconds between steps

// Step state 
bool stepState = false;  // LOW or HIGH

void setup() {
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, HIGH); // forward direction
  digitalWrite(STEP_PIN, LOW); // start LOW
}

void loop() {
  unsigned long currentMillis = millis();

  // Check if it's time to toggle the step pin
  if (currentMillis - previousMillis >= stepInterval) {
    previousMillis = currentMillis;

    // Toggle the step pin
    stepState = !stepState;
    digitalWrite(STEP_PIN, stepState);
  }

  // Other tasks can run here simultaneously
}