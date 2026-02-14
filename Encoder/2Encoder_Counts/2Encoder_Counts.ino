volatile long leftCount = 0;
volatile long rightCount = 0;

// Left encoder pins
const int encoderLeftA = 4;
const int encoderLeftB = 5;

// Right encoder pins
const int encoderRightA = 3;
const int encoderRightB = 2;

// -------------------- LEFT ENCODER ISR --------------------
void leftEncoderISR() {
  int A = digitalRead(encoderLeftA);
  int B = digitalRead(encoderLeftB);

  if (A == B)
    leftCount++;
  else
    leftCount--;
}

// -------------------- RIGHT ENCODER ISR --------------------
void rightEncoderISR() {
  int A = digitalRead(encoderRightA);
  int B = digitalRead(encoderRightB);

  if (A == B)
    rightCount++;
  else
    rightCount--;
}

// -------------------- SETUP --------------------
void setup() {
  Serial.begin(9600);

  pinMode(encoderLeftA, INPUT_PULLUP);
  pinMode(encoderLeftB, INPUT_PULLUP);
  pinMode(encoderRightA, INPUT_PULLUP);
  pinMode(encoderRightB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoderLeftA), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderRightA), rightEncoderISR, RISING);

  Serial.println("Encoder Pulse Test Started...");
  Serial.println("Left\tRight");
}

// -------------------- LOOP --------------------
void loop() {
  Serial.print(leftCount);
  Serial.print("\t");
  Serial.println(rightCount);
  delay(200);
}