int irPin = 8;
int irState = 0;

void setup () {
pinMode (irPin, INPUT);
Serial. begin (9600);
}

void loop) {
irState = digitalRead(irPin);
if (irState == LOW) {
Serial.println("Object Detected!");
} else {
Serial. println ("No Object");
}

delay (200);
}