#include <ps5Controller.h>
#include <Wire.h>

#define I2C_SDA 21
#define I2C_SCL 22
#define SLAVE_ADDR 8  // Must match Arduino

unsigned long lastSend = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin(I2C_SDA, I2C_SCL);   // ESP32 as I2C Master

  ps5.begin("7c:66:ef:78:76:f0");  // your PS5 MAC
  while (!ps5.isConnected()) {
    Serial.println("Waiting for PS5 controller...");
    delay(500);
  }
  Serial.println("Ready to send data over I2C.");
}

void loop() {
  if (ps5.isConnected() && millis() - lastSend > 100) {
    // int cross    = ps5.Cross();
    // int circle   = ps5.Circle();
    // int square   = ps5.Square();
    // int triangle = ps5.Triangle();
    // int l2       = ps5.L2();
    // int r2       = ps5.R2();

    // Scale joysticks from -128..127 → 0..255
    int lx = ps5.LStickX();
    int ly = ps5.LStickY();
    int rx = ps5.RStickX();
    int ry = ps5.RStickY();

    // Debug on ESP32 monitor
    // Serial.print("Cross: "); Serial.print(cross);
    // Serial.print(" Circle: "); Serial.print(circle);
    // Serial.print(" Square: "); Serial.print(square);
    // Serial.print(" Triangle: "); Serial.print(triangle);
    // Serial.print(" | L2: "); Serial.print(l2);
    // Serial.print(" R2: "); Serial.print(r2);
    Serial.print(" | LX: "); Serial.print(lx);
    Serial.print(" LY: "); Serial.print(ly);
    Serial.print(" RX: "); Serial.print(rx);
    Serial.print(" RY: "); Serial.println(ry);

    // Send to Arduino
    Wire.beginTransmission(SLAVE_ADDR);
    // Wire.write(cross);
    // Wire.write(circle);
    // Wire.write(square);
    // Wire.write(triangle);
    // Wire.write(l2);
    // Wire.write(r2);
    Wire.write(lx);
    Wire.write(ly);
    Wire.write(rx);
    Wire.write(ry);
    Wire.endTransmission();

    lastSend = millis();
  }
}
