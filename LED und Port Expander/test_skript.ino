#include "PCF8575.h"

PCF8575 pcf8575(0x21);
#define LED_PIN P0

void setup() {
  Serial.begin(9600);
  // Set pinMode to OUTPUT
  pcf8575.pinMode(LED_PIN, OUTPUT);
  pcf8575.pinMode(MAGNETPIN, INPUT);     //wie normal, wir geben etwas in den mikrocontroller ein, daher input
  pcf8575.begin();
}

void loop() {
  pcf8575.digitalWrite(LED_PIN, LOW);
  Serial.println("LED Pin lo");
  delay(1000);
  pcf8575.digitalWrite(LED_PIN, HIGH);
  Serial.println("LED Pin high");
  delay(1000);
}