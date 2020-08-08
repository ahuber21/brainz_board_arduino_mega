#include <HX711_ADC.h>

HX711_ADC LoadCell(28, 29); // DT pin, SCK pin

void setup() {
  Serial.begin(9600);
  
  LoadCell.begin();
  LoadCell.start(2000);
  LoadCell.setCalFactor(100);

  Serial.println("initialised");
}

void loop() {
  // put your main code here, to run repeatedly:
  LoadCell.update();
  float i = LoadCell.getData();
  Serial.println(i);
  delay(100);
}
