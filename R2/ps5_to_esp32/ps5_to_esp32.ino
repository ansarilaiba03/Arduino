# include <Arduino.h>
# include <ps5Controller.h>

# define TX_PIN 17
# define RX_PIN 16

uint8_t right, left, up, down;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  ps5.begin("e8:47:3a:5a:a3:66");
  // ps5.begin("7c:66:ef:78:76:f0");
}


void loop() {
  // put your main code here, to run repeatedly:
  if (ps5.isConnected()) {
    right = ps5.Right();  
    left = ps5.Left();
    up = ps5.Up();
    down = ps5.Down();
  }

  uint8_t packet[] = {right, left, up, down};
  
  Serial1.write((uint8_t*)packet, sizeof(packet));
  // Serial.print(right), Serial.print(" "), Serial.print(left), Serial.print(" "), Serial.print(up), Serial.print(" "), Serial.println(down);

  delay(100);
}