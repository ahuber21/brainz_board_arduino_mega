/*
 * Arduino Mega 2560 firmware
 * Communication with Raspberry Pi through I2C
 * The Arduino will operate in slave mode
 */

#include <Wire.h>
#include <SPI.h>
#include <MFRC522.h>

/*
 * NFC Pins
 */

#define NFC_RST 49
#define NFC_SS 53

// NFC
static MFRC522 mfrc522(NFC_SS, NFC_RST);

void setup() {
  // Serial port initialisation
  Serial.begin(9600);

  // NFC initialisation
  SPI.begin();
  mfrc522.PCD_Init();
}

void loop() {
  // check if new card was placed
  if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
    Serial.print("NFCREAD=");
    for (byte i = 0; i < mfrc522.uid.size; i++) {
      Serial.print(mfrc522.uid.uidByte[i] < 0x10 ? "0" : "");
      Serial.print(mfrc522.uid.uidByte[i], HEX);
    } 
    Serial.println();     
  }
}
