#include <Wire.h>

void setup() {
  Serial.begin(9600);
  Wire.begin(8); 
  Wire.onReceive(receiveEvent);
  Serial.println("Arduino Ready. Listening for ESP32-data...");
}

void loop() {
  delay(100);
}

void receiveEvent(int numBytes) {
  int cross     = Wire.read();
  int circle    = Wire.read();
  int square    = Wire.read();
  int triangle  = Wire.read();

  int l2        = Wire.read();
  int r2        = Wire.read();

  int lx        = Wire.read();
  int ly        = Wire.read();
  int rx        = Wire.read();
  int ry        = Wire.read();

  Serial.print("Cross: "); Serial.print(cross);
  Serial.print(" Circle: "); Serial.print(circle);
  Serial.print(" Square: "); Serial.print(square);
  Serial.print(" Triangle: "); Serial.print(triangle);
  Serial.print(" | L2: "); Serial.print(l2);
  Serial.print(" R2: "); Serial.print(r2);
  Serial.print(" | LX: "); Serial.print(lx);
  Serial.print(" LY: "); Serial.print(ly);
  Serial.print(" RX: "); Serial.print(rx);
  Serial.print(" RY: "); Serial.println(ry);
}