const int proximityPin = 37;   // Sensor OUT pin connected to Arduino pin 2

void setup() {
  pinMode(proximityPin, INPUT);
  Serial.begin(9600);
}

void loop() {
  int sensorState = digitalRead(proximityPin);

  if (sensorState == LOW) {
    Serial.println("Object Detected");
  } else {
    Serial.println("No Object");
  }

  delay(100);
}