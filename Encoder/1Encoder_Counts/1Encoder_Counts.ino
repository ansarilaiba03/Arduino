/* 
  Connections 
  M1A - motor wire 
  M1B - motor wire 
  V+ battery +ve 
  V- bettery -ve 
  
  DIR1 - D4
  PWM1- D5 
  GND - GND 

  Encoder 
  1 & 2nd pin - empty 
  Pin 3- 5V 
  Pin 4- GND 
  Pin 5 - D2 
  Pin 6- D3 
*/

volatile long pulseCount = 0;

int encoderA = 2;
int encoderB = 3;

void setup(){
  pinMode(encoderA, INPUT_PULLUP);
  pinMode(encoderB, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(encoderA), countPulse, RISING);

  Serial.begin(9600);
}

void loop(){
  Serial.println(pulseCount);
  // delay(200);
}

void countPulse(){
 /* if(digitalRead(encoderB)==LOW){
    pulseCount++;
  } else {
    pulseCount--;
  } */

  int A = digitalRead(encoderA);
  int B = digitalRead(encoderB);

  if(A==B) pulseCount++;
  else pulseCount--;

}