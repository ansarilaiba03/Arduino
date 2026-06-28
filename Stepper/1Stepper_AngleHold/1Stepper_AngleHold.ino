// Motor pins
#define STEP_PIN 17
#define DIR_PIN 16

// Timing variables
unsigned long previousMillis = 0;
const unsigned long stepInterval = 4; // milliseconds between steps

// Step tracking
bool stepState = false;
int stepCount = 0;

// 200 steps = 360°, so 90° = 50 steps
const int TARGET_STEPS = 90;
bool motionDone = false;

void setup() {
  Serial.begin(115200);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, 0); // forward direction
  digitalWrite(STEP_PIN, LOW);
}

void loop() {
  if (motionDone) return; // Stop after 90°

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= stepInterval) {
    previousMillis = currentMillis;

    // Only pulse if we haven't reached target
    if (stepCount < TARGET_STEPS) {
      stepState = !stepState;
      digitalWrite(STEP_PIN, stepState);

      // Count only on rising edge (one full pulse = one step)
      if (stepState == HIGH) {
        stepCount++;
      }
    } else {
      digitalWrite(STEP_PIN, LOW); // ensure pin ends LOW
      motionDone = true;
      Serial.println("90° reached. Motor stopped.");
    }
  }

  // Other tasks can run here simultaneously
}