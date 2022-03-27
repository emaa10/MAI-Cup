#include "PCF8575.h"
PCF8575 PCF(0x38);
#define LED_PIN 4

void setup() {
  Serial.begin(9600);
  PCF.begin();
}

void loop() {
  //PCF.write16(1);
  digitalWrite(LED_PIN, HIGH);
  delay(1000);
  //PCF.write16(0);
  digitalWrite(LED_PIN, LOW);
  delay(1000);
}