void setup() {
  Serial.begin(9600);                    // USB Serial (PC)
  Serial1.begin(9600, SERIAL_8N1, 7, 8); // RX=7, TX=8 (from ESP32)

  Serial.println("ESP32-S3 READY");
}

void loop() {
  static char buffer[20];
  static int index = 0;

  while (Serial1.available()) {
    char c = Serial1.read();

    if (c == '\n') {
      buffer[index] = '\0';
      index = 0;

      int ly, rx;
      if (sscanf(buffer, "%d,%d", &ly, &rx) == 2) {
        Serial.print("LY: ");
        Serial.print(ly);
        Serial.print("  RX: ");
        Serial.println(rx);
      }
    }
    else if (index < sizeof(buffer) - 1) {
      buffer[index++] = c;
    }
  }
}
