#include "PCF8575.h"

PCF8575 PCF(0x21);


void setup() {
  PCF.begin();
}

void loop() {
  PCF.write(0, HIGH);
  delay(1000);
  PCF.write(0, LOW);
  delay(1000);
}