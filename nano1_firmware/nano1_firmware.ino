/*
 * Arduino Mega 2560 firmware
 * Communication with Raspberry Pi through I2C
 * The Arduino will operate in slave mode
 */

#include <Wire.h>

#include "opcodes.h"
#include "message.h"

/*
 * I2C
 */
#define I2C_ADDRESS 0x6


/*
 * Valve pins
 */
#define VLV1 13
#define VLV2 12
#define VLV3 11
#define VLV4 10

/*
 * Indicator LEDs pins
 */
#define LED1R 9
#define LED1G 8
#define LED2R 7
#define LED2G 6
#define LED3R 5
#define LED3G 4
#define LED4R 3
#define LED4G 2
 
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

  Serial.println(F("Arduino Nano ready"));
}

void loop() {
  delay(100);
}

void receive(int byteCount) {
  while (Wire.available()) {
    unsigned int op_code = Wire.read();
    Serial.print("Received opcode: 0x");
    Serial.println(op_code, HEX);

    switch (op_code) {
      case OC_VLV1_OPEN:
        digitalWrite(VLV1, LOW);
        digitalWrite(LED1G, HIGH);
        digitalWrite(LED1R, LOW);
        break;
      case OC_VLV1_CLOSE:
        digitalWrite(VLV1, HIGH);
        digitalWrite(LED1G, LOW);
        digitalWrite(LED1R, HIGH);
        break;
      case OC_VLV2_OPEN:
        digitalWrite(VLV2, LOW);
        digitalWrite(LED2G, HIGH);
        digitalWrite(LED2R, LOW);
        break;
      case OC_VLV2_CLOSE:
        digitalWrite(VLV2, HIGH);
        digitalWrite(LED2G, LOW);
        digitalWrite(LED2R, HIGH);
        break;
      case OC_VLV3_OPEN:
        digitalWrite(VLV3, LOW);
        digitalWrite(LED3G, HIGH);
        digitalWrite(LED3R, LOW);
        break;
      case OC_VLV3_CLOSE:
        digitalWrite(VLV3, HIGH);
        digitalWrite(LED3G, LOW);
        digitalWrite(LED3R, HIGH);
        break;
      case OC_VLV4_OPEN:
        digitalWrite(VLV4, LOW);
        digitalWrite(LED4G, HIGH);
        digitalWrite(LED4R, LOW);
        break;
      case OC_VLV4_CLOSE:
        digitalWrite(VLV4, HIGH);
        digitalWrite(LED4G, LOW);
        digitalWrite(LED4R, HIGH);
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
