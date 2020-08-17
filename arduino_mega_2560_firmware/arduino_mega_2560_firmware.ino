/*
   Arduino Mega 2560 firmware
   Communication with Raspberry Pi through I2C
   The Arduino will operate in slave mode
*/

#include <HX711_ADC.h>
#include <MFRC522.h>
#include <SPI.h>
#include <Wire.h>

#include "I2C_Anything.h"
#include "message.h"
#include "opcodes.h"


byte opcode = OC_NOOP;  // opcode to be processed in next loop()

/*
   I2C
*/
#define I2C_ADDRESS 0x5

/*
   NFC Pins
*/

#define NFC_RST 49
#define NFC_SS 53

/*
   NFC
*/
static MFRC522 mfrc522(NFC_SS, NFC_RST);
unsigned long nfc_last_read_time = millis();
byte nfc_last_id[10];


/*
   Flowmeter pins
*/
#define FLOW1 10
#define FLOW2 11
#define FLOW3 12
#define FLOW4 13

// flowmeter counter
#define FLOW_IDLE 0
#define FLOW_COUNTING 1
/*
   struct to save the flowmeter information
   state shows the current data-taking status, can be FLOW_IDLE or FLOW_COUNTING
   ticks1-4 are the number of measured ticks since the last poll
   state1-4 are the last measured state (low/high) to identify ticks
*/
struct f {
  byte state;
  int ticks1;
  int ticks2;
  int ticks3;
  int ticks4;
  bool state1;
  bool state2;
  bool state3;
  bool state4;
} flow;

/*
  Scales
*/
#define Sc1_DT4 22
#define Sc1_SCK4 23
#define Sc1_DT3 24
#define Sc1_SCK3 25
#define Sc1_DT2 26
#define Sc1_SCK2 27
#define Sc1_DT1 28
#define Sc1_SCK1 29

#define Sc2_DT4 30
#define Sc2_SCK4 31
#define Sc2_DT3 32
#define Sc2_SCK3 33
#define Sc2_DT2 34
#define Sc2_SCK2 35
#define Sc2_DT1 36
#define Sc2_SCK1 37

#define Sc3_DT4 38
#define Sc3_SCK4 39
#define Sc3_DT3 40
#define Sc3_SCK3 41
#define Sc3_DT2 42
#define Sc3_SCK2 43
#define Sc3_DT1 44
#define Sc3_SCK1 45

#define Sc4_DT4 46
#define Sc4_SCK4 47
#define Sc4_DT3 48
#define Sc4_SCK3 49
#define Sc4_DT2 50
#define Sc4_SCK2 51
#define Sc4_DT1 52
#define Sc4_SCK1 53

HX711_ADC scale1_adc1(Sc1_DT1, Sc1_SCK1);
HX711_ADC scale1_adc2(Sc1_DT2, Sc1_SCK2);
HX711_ADC scale1_adc3(Sc1_DT3, Sc1_SCK3);
HX711_ADC scale1_adc4(Sc1_DT4, Sc1_SCK4);

HX711_ADC scale2_adc1(Sc2_DT1, Sc2_SCK1);
HX711_ADC scale2_adc2(Sc2_DT2, Sc2_SCK2);
HX711_ADC scale2_adc3(Sc2_DT3, Sc2_SCK3);
HX711_ADC scale2_adc4(Sc2_DT4, Sc2_SCK4);

HX711_ADC scale3_adc1(Sc3_DT1, Sc3_SCK1);
HX711_ADC scale3_adc2(Sc3_DT2, Sc3_SCK2);
HX711_ADC scale3_adc3(Sc3_DT3, Sc3_SCK3);
HX711_ADC scale3_adc4(Sc3_DT4, Sc3_SCK4);

HX711_ADC scale4_adc1(Sc4_DT1, Sc4_SCK1);
HX711_ADC scale4_adc2(Sc4_DT2, Sc4_SCK2);
HX711_ADC scale4_adc3(Sc4_DT3, Sc4_SCK3);
HX711_ADC scale4_adc4(Sc4_DT4, Sc4_SCK4);

/*
   Thermistor
*/
// the value of the 'other' resistor

#define TEMP1 A0
#define TEMP2 A1
#define TEMP3 A2
#define TEMP4 A3
#define TEMP5 A4
#define TEMP6 A5
// number of samples which are averaged
#define TEMP_NUMSAMPLES 5
// The beta coefficient of the thermistor
#define BCOEFFICIENT 4250
// resistance at 25 degrees C
#define THERMISTORNOMINAL 100000
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 25
// the value of the 'other' resistor
#define SERIESRESISTOR 100000 // 100 kOhm  
int temp_samples[TEMP_NUMSAMPLES];


// scale calibration info
byte slot;     // slot to be calibrated
float factor;  // calibration factor (slope)
float offset;  // calibration offset

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

  // Reset flowmeter
  flow_reset();
  pinMode(FLOW1, INPUT);
  pinMode(FLOW2, INPUT);
  pinMode(FLOW3, INPUT);
  pinMode(FLOW4, INPUT);
  Serial.println(F("Flowmeter ready"));

  // Init Scale ADCs
  Serial.print(F("Initialise scales..."));
  scale1_adc1.begin();
  scale1_adc1.start(100);
  scale1_adc1.setCalFactor(1000);
  scale1_adc2.begin();
  scale1_adc2.start(100);
  scale1_adc2.setCalFactor(1000);
  scale1_adc3.begin();
  scale1_adc3.start(100);
  scale1_adc3.setCalFactor(1000);
  scale1_adc4.begin();
  scale1_adc4.start(100);
  scale1_adc4.setCalFactor(1000);

  scale2_adc1.begin();
  scale2_adc1.start(100);
  scale2_adc1.setCalFactor(1000);
  scale2_adc2.begin();
  scale2_adc2.start(100);
  scale2_adc2.setCalFactor(1000);
  scale2_adc3.begin();
  scale2_adc3.start(100);
  scale2_adc3.setCalFactor(1000);
  scale2_adc4.begin();
  scale2_adc4.start(100);
  scale2_adc4.setCalFactor(1000);

  scale3_adc1.begin();
  scale3_adc1.start(100);
  scale3_adc1.setCalFactor(1000);
  scale3_adc2.begin();
  scale3_adc2.start(100);
  scale3_adc2.setCalFactor(1000);
  scale3_adc3.begin();
  scale3_adc3.start(100);
  scale3_adc3.setCalFactor(1000);
  scale3_adc4.begin();
  scale3_adc4.start(100);
  scale3_adc4.setCalFactor(1000);

  scale4_adc1.begin();
  scale4_adc1.start(100);
  scale4_adc1.setCalFactor(1000);
  scale4_adc2.begin();
  scale4_adc2.start(100);
  scale4_adc2.setCalFactor(1000);
  scale4_adc3.begin();
  scale4_adc3.start(100);
  scale4_adc3.setCalFactor(1000);
  scale4_adc4.begin();
  scale4_adc4.start(100);
  scale4_adc4.setCalFactor(1000);
  Serial.println(F("done"));

  // NFC initialisation
  Serial.print(F("Init NFC..."));
  SPI.begin();
  mfrc522.PCD_Init();
  Serial.println(F("done"));

  Serial.println(F("Arduino Mega ready"));
}

void loop() {
  /*
    Process opcode
  */
  switch (opcode) {
    case OC_NOOP:
      break;
    case OC_NFC_RCV:
      delay(10);
      nfc_read();
      break;
    case OC_FLOW_MEAS:
      flow_start_measurement();
      break;
    case OC_FLOW_READ:
      flow_read();
      break;
    case OC_FLOW_STOP:
      flow.state = FLOW_IDLE;
      break;
    case OC_FLOW_RST:
      flow_reset();
      break;
    case OC_SCALES_CALIB:
      scales_calib();
      break;
    case OC_SCALES_READ:
      scales_read();
      break;
    case OC_SCALES_TARE0:
      scale1_adc1.tareNoDelay();
      scale1_adc2.tareNoDelay();
      scale1_adc3.tareNoDelay();
      scale1_adc4.tareNoDelay();
      break;
    case OC_SCALES_TARE1:
      scale2_adc1.tareNoDelay();
      scale2_adc2.tareNoDelay();
      scale2_adc3.tareNoDelay();
      scale2_adc4.tareNoDelay();
      break;
    case OC_SCALES_TARE2:
      scale3_adc1.tareNoDelay();
      scale3_adc2.tareNoDelay();
      scale3_adc3.tareNoDelay();
      scale3_adc4.tareNoDelay();
      break;
    case OC_SCALES_TARE3:
      scale4_adc1.tareNoDelay();
      scale4_adc2.tareNoDelay();
      scale4_adc3.tareNoDelay();
      scale4_adc4.tareNoDelay();
      break;
    case OC_SCALES_RESET0:
      scale1_adc1.setCalFactor(1000);
      scale1_adc1.setTareOffset(0);
      scale1_adc2.setCalFactor(1000);
      scale1_adc2.setTareOffset(0);
      scale1_adc3.setCalFactor(1000);
      scale1_adc3.setTareOffset(0);
      scale1_adc4.setCalFactor(1000);
      scale1_adc4.setTareOffset(0);
      break;
    case OC_SCALES_RESET1:
      scale2_adc1.setCalFactor(1000);
      scale2_adc1.setTareOffset(0);
      scale2_adc2.setCalFactor(1000);
      scale2_adc2.setTareOffset(0);
      scale2_adc3.setCalFactor(1000);
      scale2_adc3.setTareOffset(0);
      scale2_adc4.setCalFactor(1000);
      scale2_adc4.setTareOffset(0);
      break;
    case OC_SCALES_RESET2:
      scale3_adc1.setCalFactor(1000);
      scale3_adc1.setTareOffset(0);
      scale3_adc2.setCalFactor(1000);
      scale3_adc2.setTareOffset(0);
      scale3_adc3.setCalFactor(1000);
      scale3_adc3.setTareOffset(0);
      scale3_adc4.setCalFactor(1000);
      scale3_adc4.setTareOffset(0);
      break;
    case OC_SCALES_RESET3:
      scale4_adc1.setCalFactor(1000);
      scale4_adc1.setTareOffset(0);
      scale4_adc2.setCalFactor(1000);
      scale4_adc2.setTareOffset(0);
      scale4_adc3.setCalFactor(1000);
      scale4_adc3.setTareOffset(0);
      scale4_adc4.setCalFactor(1000);
      scale4_adc4.setTareOffset(0);
      break;
    case OC_THERM_READ1:
      temperature_read(1);
      break;
    case OC_THERM_READ2:
      temperature_read(2);
      break;
    case OC_THERM_READ3:
      temperature_read(3);
      break;
    case OC_THERM_READ4:
      temperature_read(4);
      break;
    case OC_THERM_READ5:
      temperature_read(5);
      break;
    case OC_THERM_READ6:
      temperature_read(6);
      break;
    default:
      Serial.print(F("ERR:opcode_UNKNOWN="));
      Serial.println(opcode);
      break;
  }

  // reset opcode
  opcode = OC_NOOP;

  /*
     Count, if flowmeteres are active
  */
  if (flow.state == FLOW_COUNTING) {
    bool old_state1 = flow.state1;
    bool old_state2 = flow.state2;
    bool old_state3 = flow.state3;
    bool old_state4 = flow.state4;
    // read all pins and increment counter if the state has changed from low
    // to high
    flow.state1 = digitalRead(FLOW1);
    flow.state2 = digitalRead(FLOW2);
    flow.state3 = digitalRead(FLOW3);
    flow.state4 = digitalRead(FLOW4);
    // read again if we're high, avoid glitches
    if ((flow.state1 == HIGH) || (flow.state2 == HIGH) ||
        (flow.state3 == HIGH) || (flow.state4 == HIGH)) {
      flow.state1 = digitalRead(FLOW1);
      flow.state2 = digitalRead(FLOW2);
      flow.state3 = digitalRead(FLOW3);
      flow.state4 = digitalRead(FLOW4);
    }

    if ((old_state1 == LOW) && (flow.state1 == HIGH)) {
      flow.ticks1++;
      // Serial.println("TICK1");
    }
    if ((old_state2 == LOW) && (flow.state2 == HIGH)) {
      flow.ticks2++;
      // Serial.println("TICK2");
    }
    if ((old_state3 == LOW) && (flow.state3 == HIGH)) {
      flow.ticks3++;
      // Serial.println("TICK3");
    }
    if ((old_state4 == LOW) && (flow.state4 == HIGH)) {
      flow.ticks4++;
      // Serial.println("TICK4");
    }

    old_state1 = flow.state1;
    old_state2 = flow.state2;
    old_state3 = flow.state3;
    old_state4 = flow.state4;
  }
  delay(50);
}

void receive(int byteCount) {
  while (Wire.available()) {
    opcode = Wire.read();
    //    Serial.print("Received opcode: 0x");
    //    Serial.print(opcode, HEX);
    //    Serial.print(F(" (byteCount = "));
    //    Serial.print(byteCount);
    //    Serial.println(F(")"));

    switch (opcode) {
      // here only op codes where additional reads are required are processed
      // the rest is processed in the loop
      case OC_SCALES_CALIB:
        if (byteCount == 11) {
          // flush one byte that's somehow garbage
          Wire.read();
          slot = Wire.read();
          I2C_readAnything(factor);
          I2C_readAnything(offset);
        } else {
          Serial.println(F("I2C communication problem"));
        }
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

void scales_calib() {
  Serial.print(F("Received calib data. slot = "));
  Serial.print(slot);
  Serial.print(F("; y = "));
  Serial.print(factor);
  Serial.print(F(" * x + "));
  Serial.println(offset);

  switch (slot) {
    case 0:
      scale1_adc1.setCalFactor(factor * scale1_adc1.getCalFactor());
      scale1_adc1.setTareOffset(offset);
      scale1_adc2.setCalFactor(factor * scale1_adc2.getCalFactor());
      scale1_adc2.setTareOffset(offset);
      scale1_adc3.setCalFactor(factor * scale1_adc3.getCalFactor());
      scale1_adc3.setTareOffset(offset);
      scale1_adc4.setCalFactor(factor * scale1_adc4.getCalFactor());
      scale1_adc4.setTareOffset(offset);
      break;
    case 1:
      scale2_adc1.setCalFactor(factor * scale2_adc1.getCalFactor());
      scale2_adc1.setTareOffset(offset);
      scale2_adc2.setCalFactor(factor * scale2_adc2.getCalFactor());
      scale2_adc2.setTareOffset(offset);
      scale2_adc3.setCalFactor(factor * scale2_adc3.getCalFactor());
      scale2_adc3.setTareOffset(offset);
      scale2_adc4.setCalFactor(factor * scale2_adc4.getCalFactor());
      scale2_adc4.setTareOffset(offset);
      break;
    case 2:
      scale3_adc1.setCalFactor(factor * scale3_adc1.getCalFactor());
      scale3_adc1.setTareOffset(offset);
      scale3_adc2.setCalFactor(factor * scale3_adc2.getCalFactor());
      scale3_adc2.setTareOffset(offset);
      scale3_adc3.setCalFactor(factor * scale3_adc3.getCalFactor());
      scale3_adc3.setTareOffset(offset);
      scale3_adc4.setCalFactor(factor * scale3_adc4.getCalFactor());
      scale3_adc4.setTareOffset(offset);
      break;
    case 3:
      scale4_adc1.setCalFactor(factor * scale4_adc1.getCalFactor());
      scale4_adc1.setTareOffset(offset);
      scale4_adc2.setCalFactor(factor * scale4_adc2.getCalFactor());
      scale4_adc2.setTareOffset(offset);
      scale4_adc3.setCalFactor(factor * scale4_adc3.getCalFactor());
      scale4_adc3.setTareOffset(offset);
      scale4_adc4.setCalFactor(factor * scale4_adc4.getCalFactor());
      scale4_adc4.setTareOffset(offset);
      break;
    default:
      Serial.print(F("Bad slot: "));
      Serial.println(slot);
      break;
  }
}

void scales_read() {
  float val1 = 0;
  scale1_adc1.update();
  val1 += scale1_adc1.getData();
  scale1_adc2.update();
  val1 += scale1_adc2.getData();
  scale1_adc3.update();
  val1 += scale1_adc3.getData();
  scale1_adc4.update();
  val1 += scale1_adc4.getData();

  float val2 = 0;
  scale2_adc1.update();
  val2 += scale2_adc1.getData();
  scale2_adc2.update();
  val2 += scale2_adc2.getData();
  scale2_adc3.update();
  val2 += scale2_adc3.getData();
  scale2_adc4.update();
  val2 += scale2_adc4.getData();

  float val3 = 0;
  scale3_adc1.update();
  val3 += scale3_adc1.getData();
  scale3_adc2.update();
  val3 += scale3_adc2.getData();
  scale3_adc3.update();
  val3 += scale3_adc3.getData();
  scale3_adc4.update();
  val3 += scale3_adc4.getData();

  float val4 = 0;
  scale4_adc1.update();
  val4 += scale4_adc1.getData();
  scale4_adc2.update();
  val4 += scale4_adc2.getData();
  scale4_adc3.update();
  val4 += scale4_adc3.getData();
  scale4_adc4.update();
  val4 += scale4_adc4.getData();

  Serial.print(F("SCALE_READ: val1 = "));
  Serial.print(val1);
  Serial.print(F(", val2 = "));
  Serial.print(val2);
  Serial.print(F(", val3 = "));
  Serial.print(val3);
  Serial.print(F(", val4 = "));
  Serial.println(val4);

  msg_reset();
  msg_put("WEIGHT=");
  msg_put(val1);
  msg_put(" ");
  msg_put(val2);
  msg_put(" ");
  msg_put(val3);
  msg_put(" ");
  msg_put(val4);
  message.ready = true;
}

void flow_start_measurement() {
  // Serial.println(F("Starting flowmeter measurement"));
  flow_reset();
  flow.state = FLOW_COUNTING;
}

void flow_read() {
  // Serial.println(F("Readout flowmeter"));
  msg_reset();
  msg_put("FLOW=");
  msg_put(flow.ticks1);
  msg_put(" ");
  msg_put(flow.ticks2);
  msg_put(" ");
  msg_put(flow.ticks3);
  msg_put(" ");
  msg_put(flow.ticks4);
  flow_reset();
  message.ready = true;
}

void flow_reset() {
  flow.state = FLOW_IDLE;
  flow.ticks1 = 0;
  flow.ticks2 = 0;
  flow.ticks3 = 0;
  flow.ticks4 = 0;
  flow.state1 = digitalRead(FLOW1);
  flow.state2 = digitalRead(FLOW2);
  flow.state3 = digitalRead(FLOW3);
  flow.state4 = digitalRead(FLOW4);
}

void nfc_read() {
  msg_reset();
  msg_put("NFCREAD=");
  if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
    memset(nfc_last_id, 0, 10);  // reset last id
    for (byte i = 0; i < mfrc522.uid.size; i++) {
      char buf[4];
      memset(buf, 0, 4);
      sprintf(buf, "%02x", mfrc522.uid.uidByte[i]);
      nfc_last_id[i] = mfrc522.uid.uidByte[i];
      msg_put(buf);
    }
    nfc_last_read_time = millis();
  } else if (millis() - nfc_last_read_time < 500) {
    // take the last one again
    // mainly there to avoid quick changes between valid ID and 0
    for (byte i = 0; i < mfrc522.uid.size; i++ ) {
      char buf[4];
      memset(buf, 0, 4);
      sprintf(buf, "%02x", nfc_last_id[i]);
      msg_put(buf);
    }
  } else {
    msg_put("0");
  }
  message.ready = true;
}

void temperature_read(byte id) {
  msg_reset();

  int pin = -1;
  switch (id) {
    case 1:
      pin = TEMP1;
      break;
    case 2:
      pin = TEMP2;
      break;
    case 3:
      pin = TEMP3;
      break;
    case 4:
      pin = TEMP4;
      break;
    case 5:
      pin = TEMP5;
      break;
    case 6:
      pin = TEMP6;
      break;
  }

  uint8_t i;
  float average;

  // take N samples in a row, with a slight delay
  for (i = 0; i < TEMP_NUMSAMPLES; i++) {
    temp_samples[i] = analogRead(pin);
    delay(10);
  }

  // average all the samples out
  average = 0;
  for (i = 0; i < TEMP_NUMSAMPLES; i++) {
    average += temp_samples[i];
  }
  average /= TEMP_NUMSAMPLES;

  // convert the value to resistance
  average = 1023 / average - 1;
  average = SERIESRESISTOR * average;

  float steinhart;
  steinhart = average / THERMISTORNOMINAL;     // (R/Ro)
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                 // Invert
  steinhart -= 273.15;                         // convert to C

  msg_put("TEMP=");
  msg_put(steinhart);
  message.ready = true;
}
