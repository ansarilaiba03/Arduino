int LED1 = 13;
int LED2 = 7;
int LED3 = 3;
int LED4 = 4;

void setup() {
pinMode(LED1, OUTPUT);
pinMode(LED2, OUTPUT);
pinMode(LED3, OUTPUT);
pinMode(LED4, OUTPUT);
}

void loop() {
  digitalWrite(LED1, HIGH);  
  delay(200);                
  digitalWrite(LED1, LOW);   
  delay(200);   
  digitalWrite(LED2, HIGH);  
  delay(200);                
  digitalWrite(LED2, LOW);   
  delay(200);                
  digitalWrite(LED3, HIGH);  
  delay(200);                
  digitalWrite(LED3, LOW);   
  delay(200);
  digitalWrite(LED4, HIGH);  
  delay(200);                
  digitalWrite(LED4, LOW);   
  delay(200);                    
}

