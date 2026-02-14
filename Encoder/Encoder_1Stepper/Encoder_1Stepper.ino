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

#define ENCODER_CLK 2
#define ENCODER_DT 4

#define DIR_PIN 7
#define PUL_PIN 6

volatile bool lastA = LOW;
volatile float pulseCounter = 0.0 ;

void setup() {
  pinMode(ENCODER_CLK, INPUT_PULLUP);
  pinMode(ENCODER_DT, INPUT_PULLUP);

  pinMode(DIR_PIN, OUTPUT);
  pinMode(PUL_PIN, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_CLK), readEncoder, CHANGE);
}

void loop() {
  if(pulseCounter != 0){
    digitalWrite(DIR_PIN, pulseCounter > 0 ? HIGH : LOW);

    digitalWrite(PUL_PIN, HIGH);
    delay(1);
    digitalWrite(PUL_PIN, LOW);
    delay(1);

    // decrease counter toward zero
    pulseCounter += (pulseCounter > 0 ? -1 : 1);
  }
}
void readEncoder(){
  bool currentA = digitalRead(ENCODER_CLK);
  bool currentB = digitalRead(ENCODER_DT);

  if(currentA != lastA){
    float stepDir = (currentA == currentB) ? 1 : -1;

    pulseCounter += stepDir / 2.0 ;  // divide by 2 because 2 edges per pulse
  }

  lastA = currentA;
}