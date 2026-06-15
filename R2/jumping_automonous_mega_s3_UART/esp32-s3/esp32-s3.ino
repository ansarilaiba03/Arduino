#include <Arduino.h>
#include <ESP32Servo.h>

// =====================================
// PINS
// =====================================
const int bldc_pin       = 12;
const int relay_pin      = 13;
const int front_pneu_pin = 18;
const int rear_pneu_pin  = 17;
const int enc_A          = 4;
const int enc_B          = 5;

// =====================================
// BLDC
// =====================================
Servo bldc;
const int BLDC_STOP    = 1000;
const int BLDC_SPEED   = 1600;   // tune this

// =====================================
// BLDC ENCODER
// =====================================
volatile long bldc_count = 0;

void IRAM_ATTR readEncoder()
{
  if (digitalRead(enc_A) == digitalRead(enc_B)) bldc_count++;
  else bldc_count--;
}

// =====================================
// UART — RX=11, TX=10
// =====================================
const uint8_t CMD_OPEN_ALL_PNEU    = 0x01;
const uint8_t CMD_CLOSE_FRONT_PNEU = 0x02;
const uint8_t CMD_CLOSE_REAR_PNEU  = 0x03;
const uint8_t CMD_BLDC_CCW         = 0x04;

const uint8_t ACK_PNEU_OPENED    = 0x11;
const uint8_t ACK_BLDC_UP_DONE   = 0x12;
const uint8_t ACK_FRONT_CLOSED   = 0x13;
const uint8_t ACK_REAR_CLOSED    = 0x14;
const uint8_t ACK_BLDC_DOWN_DONE = 0x15;

// =====================================
// STATES
// =====================================
enum ESP_State
{
  ESP_IDLE,
  ESP_OPEN_PNEUMATICS_AND_BLDC_UP,
  ESP_CLOSE_FRONT,
  ESP_CLOSE_REAR,
  ESP_BLDC_FLIP_AND_DOWN,
  ESP_DONE
};

ESP_State espState = ESP_IDLE;

// =====================================
// SETUP
// =====================================
void setup()
{
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, 11, 10); 

  pinMode(relay_pin,      OUTPUT);
  pinMode(front_pneu_pin, OUTPUT);
  pinMode(rear_pneu_pin,  OUTPUT);

  digitalWrite(relay_pin,      LOW);
  digitalWrite(front_pneu_pin, LOW);
  digitalWrite(rear_pneu_pin,  LOW);

  pinMode(enc_A, INPUT_PULLUP);
  pinMode(enc_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(enc_A), readEncoder, RISING);

  bldc.setPeriodHertz(50);
  bldc.attach(bldc_pin, 500, 2400);
  bldc.writeMicroseconds(BLDC_STOP);

  delay(3000);
  Serial.println("ESP32-S3 ready");
}

// =====================================
// LOOP
// =====================================
void loop()
{
  // Read command from Mega
  if (Serial1.available())
  {
    uint8_t cmd = Serial1.read();
    Serial.print("CMD: 0x"); Serial.println(cmd, HEX);

    if      (cmd == CMD_OPEN_ALL_PNEU)    espState = ESP_OPEN_PNEUMATICS_AND_BLDC_UP;
    else if (cmd == CMD_CLOSE_FRONT_PNEU) espState = ESP_CLOSE_FRONT;
    else if (cmd == CMD_CLOSE_REAR_PNEU)  espState = ESP_CLOSE_REAR;
    else if (cmd == CMD_BLDC_CCW)         espState = ESP_BLDC_FLIP_AND_DOWN;
  }

  Serial.print("bldc_count="); Serial.print(bldc_count);
  Serial.print(" STATE=");     Serial.println(espState);

  // =====================================
  // STATE: ESP_IDLE
  // Waiting for command from Mega
  // =====================================
  if (espState == ESP_IDLE)
  {
    // Nothing to do — waiting
  }

  // =====================================
  // STATE: ESP_OPEN_PNEUMATICS_AND_BLDC_UP
  // Opens pneumatics AND starts BLDC
  // together in the same state
  // Waits until BLDC hits 500 counts
  // Sends one single ACK to Mega
  // =====================================
  else if (espState == ESP_OPEN_PNEUMATICS_AND_BLDC_UP)
  {
    // Open both pneumatics immediately
    digitalWrite(front_pneu_pin, HIGH);
    digitalWrite(rear_pneu_pin,  HIGH);
    Serial.println("All pneumatics OPEN");

    // Start BLDC at the same time
    bldc_count = 0;
    bldc.writeMicroseconds(BLDC_SPEED);
    Serial.println("BLDC CW started");

    // Now wait here until BLDC encoder hits 500
    while (abs(bldc_count) < 500)
    {
      delay(5);
    }

    // Both done — stop BLDC, send single ACK
    bldc.writeMicroseconds(BLDC_STOP);
    Serial.println("BLDC at 500 counts — all ready");

    Serial1.write(ACK_READY);
    espState = ESP_IDLE;
  }

  // =====================================
  // STATE: ESP_CLOSE_FRONT
  // Retract front pneumatics
  // =====================================
  else if (espState == ESP_CLOSE_FRONT)
  {
    digitalWrite(front_pneu_pin, LOW);
    Serial.println("Front pneumatics CLOSED");
    Serial1.write(ACK_FRONT_CLOSED);
    espState = ESP_IDLE;
  }

  // =====================================
  // STATE: ESP_CLOSE_REAR
  // Retract rear pneumatics
  // =====================================
  else if (espState == ESP_CLOSE_REAR)
  {
    digitalWrite(rear_pneu_pin, LOW);
    Serial.println("Rear pneumatics CLOSED");
    Serial1.write(ACK_REAR_CLOSED);
    espState = ESP_IDLE;
  }

  // =====================================
  // STATE: ESP_BLDC_FLIP_AND_DOWN
  // Flip relay to change BLDC phase
  // Run anticlockwise back to 0 counts
  // =====================================
  else if (espState == ESP_BLDC_FLIP_AND_DOWN)
  {
    // Flip relay (changes motor phase → now CCW)
    digitalWrite(relay_pin, HIGH);
    delay(200);

    bldc.writeMicroseconds(BLDC_SPEED);
    Serial.println("BLDC CCW started");

    // Wait until encoder returns to 0
    while (abs(bldc_count) > 0)
    {
      delay(5);
    }

    bldc.writeMicroseconds(BLDC_STOP);
    digitalWrite(relay_pin, LOW);
    Serial.println("BLDC DOWN done — back to 0");

    Serial1.write(ACK_BLDC_DOWN_DONE);
    espState = ESP_DONE;
  }

  // =====================================
  // STATE: ESP_DONE
  // All done — idle forever
  // =====================================
  else if (espState == ESP_DONE)
  {
    // Sequence complete
  }

  delay(10);
}