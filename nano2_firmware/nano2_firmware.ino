/*
   Arduino Mega 2560 firmware
   Communication with Raspberry Pi through I2C
   The Arduino will operate in slave mode
*/

#include <Wire.h>

#include "opcodes.h"
#include "message.h"
#include "I2C_Anything.h"

/*
   I2C
*/
#define I2C_ADDRESS 0x7

#define ON_TIME 1  // number of millis the segments will light up

//ABCDEFG,dp
const int numeral[10] = {
  B11111100, //0
  B01100000, //1
  B11011010, //2
  B11110010, //3
  B01100110, //4
  B10110110, //5
  B10111110, //6
  B11100000, //7
  B11111110, //8
  B11110110, //9
};
const int idle_bit[12] = {
  B00000100,
  B00001000,
  B00010000,
  B00010000,
  B00010000,
  B00010000,
  B00100000,
  B01000000,
  B10000000,
  B10000000,
  B10000000,
  B10000000
};

//pins for decimal point and each segment
//dp, G, F, E, D, C, B, A
const int segmentPins[] = { 1, 8, 7, 6, 5, 4, 3, 2};

const int numberofDigits = 4;

const int digitPins[numberofDigits] = { 12, 11, 10, 9}; //digits 1, 2, 3, 4

byte opcode;
int current_number;
byte state;

#define IDLE_DELAY 140  // time each segment lights up
byte idle_state;
long int idle_last_time;

// is the display on?
bool on;

void setup()
{
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

  Serial.println(F("Init segments"));
  for (int i = 0; i < 8; i++)
    pinMode(segmentPins[i], OUTPUT); //set segment and DP pins to output

  //sets the digit pins as outputs
  for (int i = 0; i < numberofDigits; i++)
    pinMode(digitPins[i], OUTPUT);

  on = false;
  state = 0;
  idle_state = 0;
  idle_last_time = 0;

  Serial.println(F("Arduino Nano ready"));
}

void loop()
{
  if (on) {
    switch (state) {
      case 0:
        idle();
        break;
      case 1:
        showNumber(current_number);
        break;
    }
  }
}

void receive(int byteCount) {
  while (Wire.available()) {
    opcode = Wire.read();
    Serial.print("Received opcode: 0x");
    Serial.print(opcode, HEX);
    Serial.print(F(" (byteCount = "));
    Serial.print(byteCount);
    Serial.println(F(")"));

    switch (opcode) {
      // here only op codes where additional reads are required are processed
      // the rest is processed in the loop
      case OC_DISPLAY_SHOW:
          // flush one byte that's somehow garbage
          state = 1; // show number
          Wire.read();
          I2C_readAnything(current_number);
          Serial.print(F("Received number: "));
          Serial.println(current_number);
        break;
      case OC_DISPLAY_IDLE:
        state = 0;
        break;
      case OC_DISPLAY_ON:
        on = true;
        break;
      case OC_DISPLAY_OFF:
        on = false;
        break;
      default:
        break;
    }
  }
}

void request() {
  if (message.ready) {
    Wire.write(message.text[message.idx++]);
    if (message.idx == message.len) {
      message.ready = false;
      Wire.write(0);
      Serial.print(message.text);
      Serial.print(F(" ["));
      Serial.print(message.len);
      Serial.println(F(" bytes]"));
    }
  } else {
    Wire.write(0);
  }
}

void showNumber (int number)
{
  if (number == 0)
  {
    showDigit(0, numberofDigits - 1); //display 0 in the rightmost digit
  } else
  {
    for (int digit = numberofDigits - 1; digit >= 0; digit--)
    {
      if (number > 0)
      {
        showDigit(number % 10, digit);
        number = number / 10;
      }
    }
  }
}

//Displays given number on a 7-segment display at the given digit position
void showDigit (int number, int digit)
{
  for (int segment = 1; segment < 8; segment++)
  {
    boolean isBitSet = bitRead(numeral[number], segment);
    digitalWrite(segmentPins[segment], isBitSet);
  }
  digitalWrite(digitPins[digit], HIGH);
  delay(ON_TIME);
  digitalWrite(digitPins[digit], LOW);
}

void idle() {
  if ((millis() - idle_last_time) > IDLE_DELAY) {
    ++idle_state;
    idle_state = idle_state % 12;
    idle_last_time = millis();
  }
  byte digitPin = 0;
  switch (idle_state) {
    case 0:
      digitPin = digitPins[0];
      break;
    case 1:
      digitPin = digitPins[0];
      break;
    case 2:
      digitPin = digitPins[0];
      break;
    case 3:
      digitPin = digitPins[1];
      break;
    case 4:
      digitPin = digitPins[2];
      break;
    case 5:
      digitPin = digitPins[3];
      break;
    case 6:
      digitPin = digitPins[3];
      break;
    case 7:
      digitPin = digitPins[3];
      break;
    case 8:
      digitPin = digitPins[3];
      break;
    case 9:
      digitPin = digitPins[2];
      break;
    case 10:
      digitPin = digitPins[1];
      break;
    case 11:
      digitPin = digitPins[0];
      break;
    default:
      break;
  }
  for (int segment = 1; segment < 8; segment++)
  {
    boolean isBitSet = bitRead(idle_bit[idle_state], segment);
    digitalWrite(segmentPins[segment], isBitSet);
  }
  digitalWrite(digitPin, HIGH);
  delay(ON_TIME);
  digitalWrite(digitPin, LOW);
}
