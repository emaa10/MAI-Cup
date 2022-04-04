#include "PCF8575.h"

PCF8575(0x21);

void setup() {
  pcf8575.pinMode(P0, OUTPUT);
}

void loop() {
  PCF8575.digitalWrite(P0, HIGH);
  delay(1000);
  PCF8575.digitalWrite(P0, LOW);
  delay(1000);
}