#include <ps5Controller.h>

#define RXD2 16
#define TXD2 17

unsigned long lastSend = 0;

void setup() {
  Serial.begin(9600);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);  // UART to Arduino Mega

  ps5.begin("7C:66:EF:78:76:F0"); // your PS5 MAC

  Serial.println("Connecting PS5...");
  while (!ps5.isConnected()) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nPS5 connected!");
}
 
void loop() {
  if (ps5.isConnected() && millis() - lastSend > 100) {
    int ly = ps5.LStickY();   // forward/backward
    int rx = ps5.RStickX();   // turning

    // Debug: print to Serial Monitor
    Serial.print("LY: "); Serial.print(ly);
    Serial.print("  RX: "); Serial.println(rx);

    // Send to Mega via UART
    Serial2.print(ly); Serial2.print(",");
    Serial2.println(rx);

    lastSend = millis();
  }
  if (ps5.isConnected()) {
    if (ps5.Circle()) { // When O button is pressed
      Serial.println("Restart command sent!");
      Serial2.println("R");  // Send restart signal to Arduino
      delay(500);            // Prevent rapid re-triggers
    }
  }
}
