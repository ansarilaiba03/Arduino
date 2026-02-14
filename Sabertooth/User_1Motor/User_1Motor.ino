//user
#include <SabertoothSimplified.h>

SabertoothSimplified ST;

void setup() {
  
  SabertoothTXPinSerial.begin(9600);
}

void loop() {
  ST.motor(1, 127);
  ST.motor(2, 127); 
  delay(2000);
}