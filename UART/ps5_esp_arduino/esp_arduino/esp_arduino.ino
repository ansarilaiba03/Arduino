/* uno
#include <SoftwareSerial.h>

// RX = Arduino pin connected to ESP32 TX
// TX = Arduino pin connected to ESP32 RX (optional)
SoftwareSerial ESPSerial(10,11); // RX, TX

void setup() {
  Serial.begin(9600);      // USB Serial Monitor
  ESPSerial.begin(9600);   // UART from ESP32
  Serial.println("Arduino ready. Listening for PS5 data via UART...");
}

void loop() {
  if (ESPSerial.available()) {
    // Read one line of data from ESP32
    String data = ESPSerial.readStringUntil('\n');
    //Serial.println("PS5 Data: " + data);
    delay(100);

    // Parse comma-separated values into array
    int values[10]; // cross, circle, square, triangle, l2, r2, lx, ly, rx, ry
    int index = 0;
    int start = 0;

    for (int i = 0; i < data.length(); i++) {
      if (data[i] == ',' || i == data.length() - 1) {
        values[index++] = data.substring(start, i + 1).toInt();
        start = i + 1;
      }
    }

    // Now you can use the values directly:
    // values[0] = Cross, values[1] = Circle, ..., values[9] = RY
    Serial.print("Cross: "); Serial.print(values[0]);
    Serial.print(" Circle: "); Serial.print(values[1]);
    Serial.print(" Square: "); Serial.print(values[2]);
    Serial.print(" Triangle: "); Serial.print(values[3]);
    Serial.print(" L2: "); Serial.print(values[4]);
    Serial.print(" R2: "); Serial.print(values[5]);
    Serial.print(" LX: "); Serial.print(values[6]);
    Serial.print(" LY: "); Serial.print(values[7]);
    Serial.print(" RX: "); Serial.print(values[8]);
    Serial.print(" RY: "); Serial.println(values[9]);
  }
}
*/











// mega
void setup() {
  Serial.begin(9600);     // Debug to PC
  Serial1.begin(9600);    // UART from ESP32
  Serial.println("Mega ready. Waiting for PS5 data...");
}

void loop() {
  if (Serial1.available()) {
    String data = Serial1.readStringUntil('\n');
    int values[10];
    int index = 0;

    char buffer[data.length() + 1];
    data.toCharArray(buffer, sizeof(buffer));
    char *token = strtok(buffer, ",");

    while (token != NULL && index < 10) {
      values[index++] = atoi(token);
      token = strtok(NULL, ",");
    }

    // Print received values
    Serial.print("LX: "); Serial.print(values[6]);
    Serial.print(" LY: "); Serial.print(values[7]);
    Serial.print(" RX: "); Serial.print(values[8]);
    Serial.print(" RY: "); Serial.println(values[9]);
  }
}