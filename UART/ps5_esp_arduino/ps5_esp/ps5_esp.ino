#include <ps5Controller.h>

#define RXp2 16
#define TXp2 17

ps5Controller ps;  // Create the PS5Controller object

unsigned long lastSend = 0;

void setup() {
  Serial.begin(9600);                      
  Serial2.begin(9600, SERIAL_8N1, RXp2, TXp2); // UART to Arduino

  // Connect using PS5 MAC address
  ps.begin("7C:66:EF:78:76:F0"); // Replace with your PS5 controller MAC

  Serial.println("Waiting for PS5 controller to connect via Bluetooth...");
  while (!ps.isConnected()) {
    delay(500);
  }
  Serial.println("PS5 Connected! Ready to send data over UART.");
}

void loop() {
  if (ps.isConnected() && millis() - lastSend > 300) {
    // Read buttons
    int cross    = ps.Cross();
    int circle   = ps.Circle();
    int square   = ps.Square();
    int triangle = ps.Triangle();
    int l2       = ps.L2();
    int r2       = ps.R2();

    // Read joysticks
    int lx = ps.LStickX();
    int ly = ps.LStickY();
    int rx = ps.RStickX();
    int ry = ps.RStickY();

    // Debug on ESP32 monitor
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

    // Send data to Arduino via UART (comma-separated)
    Serial2.print(cross); Serial2.print(",");
    Serial2.print(circle); Serial2.print(",");
    Serial2.print(square); Serial2.print(",");
    Serial2.print(triangle); Serial2.print(",");
    Serial2.print(l2); Serial2.print(",");
    Serial2.print(r2); Serial2.print(",");
    Serial2.print(lx); Serial2.print(",");
    Serial2.print(ly); Serial2.print(",");
    Serial2.print(rx); Serial2.print(",");
    Serial2.println(ry);

    lastSend = millis();
  }
}