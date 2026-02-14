// Motor 1 pins
#define M1_STEP 3
#define M1_DIR  2

// Motor 2 pins
#define M2_STEP 5
#define M2_DIR  4

// Motor parameters
const float stepsPerRevolution = 200.0;   // 1.8° per step
const float microsteps = 16.0;            // Driver microstepping
const float totalStepsPerRev = stepsPerRevolution * microsteps;

// Target angle (modify this)
float targetAngle = 45.0;  // degrees
float stepsToMove;           // will be calculated in setup()

// Timing control
unsigned long previousMicros = 0;
const unsigned long stepDelayMicros = 1000;  // Lower = faster speed

// Step tracking
float currentStep = 0.0;
bool movementDone = false;

void setup() {
  pinMode(M1_STEP, OUTPUT);
  pinMode(M1_DIR, OUTPUT);
  pinMode(M2_STEP, OUTPUT);
  pinMode(M2_DIR, OUTPUT);

  // Direction (HIGH = forward)
  digitalWrite(M1_DIR, HIGH);
  digitalWrite(M2_DIR, HIGH);

  digitalWrite(M1_STEP, LOW);
  digitalWrite(M2_STEP, LOW);

  // Calculate steps to move based on angle
  stepsToMove = (targetAngle / 360.0) * totalStepsPerRev;

  Serial.begin(9600);
  Serial.print("Target steps to move: ");
  Serial.println(stepsToMove);
}

void loop() {
  if (movementDone) {
    return;  // Hold position
  }

  unsigned long currentMicros = micros();

  if (currentStep < stepsToMove && (currentMicros - previousMicros >= stepDelayMicros)) {
    previousMicros = currentMicros;

    // Step pulse
    digitalWrite(M1_STEP, HIGH);
    digitalWrite(M2_STEP, HIGH);
    delayMicroseconds(5);
    digitalWrite(M1_STEP, LOW);
    digitalWrite(M2_STEP, LOW);

    currentStep++;
  }

  if (currentStep >= stepsToMove && !movementDone) {
    movementDone = true;
    Serial.println("Target angle reached. Holding position.");
  }
}
