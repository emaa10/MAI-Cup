#include "PCF8575.h"
PCF8575 pcf8575(0x21, A0, A1);
#define LED_PIN P0

void setup() {
  Serial.begin(9600);
  pcf8575.begin();
}

void loop() {
  pcf8575.digitalWrite(LED_PIN, HIGH);
  delay(1000);
  pcf8575.digitalWrite(LED_PIN, LOW);
  delay(1000);
}