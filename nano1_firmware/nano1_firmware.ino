/*
   Arduino Nano 1firmware
   Communication with Raspberry Pi through I2C
   The Arduino will operate in slave mode
*/

#include <Wire.h>

#include "opcodes.h"
#include "message.h"

/*
   I2C
*/
#define I2C_ADDRESS 0x6


/*
   Valve pins
*/
#define VLV1 13
#define VLV2 12
#define VLV3 11
#define VLV4 10

/*
   Indicator LEDs pins
*/
#define LED1R 9
#define LED1G 8
#define LED2R 7
#define LED2G 6
#define LED3R 5
#define LED3G 4
#define LED4R 3
#define LED4G 2

bool STATE1;
bool STATE2;
bool STATE3;
bool STATE4;
long idle_millis;
byte idle_idx;

void setup() {
  // Serial port initialisation
  Serial.begin(9600);
  Serial.println(F("Serial started"));

  // Configure I2C communication with Pi
  Serial.print(F("Init I2C..."));
  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(receive);
  Wire.onRequest(request);
  // initialise the message
  msg_reset();
  Serial.println(F("done"));

  // Set up pin mode for valves
  pinMode(VLV1, OUTPUT);
  pinMode(VLV2, OUTPUT);
  pinMode(VLV3, OUTPUT);
  pinMode(VLV4, OUTPUT);
  digitalWrite(VLV1, HIGH);
  digitalWrite(VLV2, HIGH);
  digitalWrite(VLV3, HIGH);
  digitalWrite(VLV4, HIGH);

  pinMode(LED1R, OUTPUT);
  pinMode(LED1G, OUTPUT);
  pinMode(LED2R, OUTPUT);
  pinMode(LED2G, OUTPUT);
  pinMode(LED3R, OUTPUT);
  pinMode(LED3G, OUTPUT);
  pinMode(LED4R, OUTPUT);
  pinMode(LED4G, OUTPUT);

  digitalWrite(LED1G, LOW);
  digitalWrite(LED2G, LOW);
  digitalWrite(LED3G, LOW);
  digitalWrite(LED4G, LOW);

  digitalWrite(LED1R, HIGH);
  digitalWrite(LED2R, HIGH);
  digitalWrite(LED3R, HIGH);
  digitalWrite(LED4R, HIGH);
  Serial.println(F("Pin modes initialised"));

  idle_millis = millis();
  idle_idx = 0;
  STATE1 = 0;
  STATE2 = 0;
  STATE3 = 0;
  STATE4 = 0;

  Serial.println(F("Arduino Nano ready"));
}

void loop() {
  digitalWrite(LED1R, LOW);
  digitalWrite(LED1G, LOW);
  digitalWrite(LED2R, LOW);
  digitalWrite(LED2G, LOW);
  digitalWrite(LED3R, LOW);
  digitalWrite(LED3G, LOW);
  digitalWrite(LED4R, LOW);
  digitalWrite(LED4G, LOW);
  if (
    STATE1 == 0
    && STATE2 == 0
    && STATE3 == 0
    && STATE4 == 0) {
    if (millis() - idle_millis > 250) {
      idle_millis = millis();
      idle_idx = (idle_idx + 1) % 6;
    }
    switch (idle_idx) {
      case 0:
        digitalWrite(LED1R, LOW);
        digitalWrite(LED1G, HIGH);
        digitalWrite(LED2R, HIGH);
        digitalWrite(LED3R, HIGH);
        digitalWrite(LED4R, HIGH);
        break;
      case 1:
        digitalWrite(LED1R, HIGH);
        digitalWrite(LED2R, LOW);
        digitalWrite(LED2G, HIGH);
        digitalWrite(LED3R, HIGH);
        digitalWrite(LED4R, HIGH);
        break;
      case 2:
        digitalWrite(LED1R, HIGH);
        digitalWrite(LED2R, HIGH);
        digitalWrite(LED3R, LOW);
        digitalWrite(LED3G, HIGH);
        digitalWrite(LED4R, HIGH);
        break;
      case 3:
        digitalWrite(LED1R, HIGH);
        digitalWrite(LED2R, HIGH);
        digitalWrite(LED3R, HIGH);
        digitalWrite(LED4R, LOW);
        digitalWrite(LED4G, HIGH);
        break;
      case 4:
        digitalWrite(LED1R, HIGH);
        digitalWrite(LED2R, HIGH);
        digitalWrite(LED3R, LOW);
        digitalWrite(LED3G, HIGH);
        digitalWrite(LED4R, HIGH);
        break;
      case 5:
        digitalWrite(LED1R, HIGH);
        digitalWrite(LED2R, LOW);
        digitalWrite(LED2G, HIGH);
        digitalWrite(LED3R, HIGH);
        digitalWrite(LED4R, HIGH);
        break;
      default:
        break;
    }
  }
  if (STATE1) {
    digitalWrite(LED1R, LOW);
    digitalWrite(LED1G, HIGH);
  }
  if (STATE2) {
    digitalWrite(LED2R, LOW);
    digitalWrite(LED2G, HIGH);
  }
  if (STATE3) {
    digitalWrite(LED3R, LOW);
    digitalWrite(LED3G, HIGH);
  }
  if (STATE4) {
    digitalWrite(LED4R, LOW);
    digitalWrite(LED4G, HIGH);
  }

  delay(5);
}

void receive(int byteCount) {
  while (Wire.available()) {
    unsigned int op_code = Wire.read();
    Serial.print("Received opcode: 0x");
    Serial.println(op_code, HEX);

    switch (op_code) {
      case OC_VLV1_OPEN:
        digitalWrite(VLV1, LOW);
        STATE1 = 1;
        break;
      case OC_VLV1_CLOSE:
        digitalWrite(VLV1, HIGH);
        STATE1 = 0;
        break;
      case OC_VLV2_OPEN:
        digitalWrite(VLV2, LOW);
        STATE2 = 1;
        break;
      case OC_VLV2_CLOSE:
        digitalWrite(VLV2, HIGH);
        STATE2 = 0;
        break;
      case OC_VLV3_OPEN:
        digitalWrite(VLV3, LOW);
        STATE3 = 1;
        break;
      case OC_VLV3_CLOSE:
        digitalWrite(VLV3, HIGH);
        STATE3 = 0;
        break;
      case OC_VLV4_OPEN:
        digitalWrite(VLV4, LOW);
        STATE4 = 1;
        break;
      case OC_VLV4_CLOSE:
        digitalWrite(VLV4, HIGH);
        STATE4 = 0;
        break;
      default:
        Serial.print(F("ERR:OP_CODE_UNKNOWN="));
        Serial.println(op_code);
        break;
    }
  }
}

void request() {
  if (message.ready) {
    Wire.write(message.text[message.idx++]);
    if (message.idx == message.len) {
      Wire.write(0);
      message.ready = false;
    }
  } else {
    Wire.write(0);
  }
  delay(10);
}
