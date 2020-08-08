#include <Wire.h>

#define ADDRESS 0x5
#define PIN 23

int number = 0;

void setup() {
  pinMode(PIN, OUTPUT);
  Serial.begin(9600);
  Wire.begin(ADDRESS);

  Wire.onReceive(receive);
  Wire.onRequest(request);
}

void loop() {
  delay(100);
}

void receive(int byteCount) {
  while (Wire.available()) {
    number = Wire.read();
    Serial.print("Received data:" );
    Serial.println(number);

    if (number == 1) {
      digitalWrite(PIN, HIGH);
    } else {
      digitalWrite(PIN, LOW);
    }
  }
} 

void request() {
  Wire.write(number);
}
