#include "PCF8575.h"
PCF8575 pcf8575(0x21);
#define LED_PIN 4

void setup() {
  Serial.begin(9600);
  pcf8575.begin();

  pcf8575.pinMode(P0, OUTPUT);  
  pcf8575.pinMode(P1, INPUT);  
  pcf8575.pinMode(P2, INPUT);  
}

void loop() {
  //PCF.write16(1);
  digitalWrite(LED_PIN, HIGH);
  delay(1000);
  //PCF.write16(0);
  digitalWrite(LED_PIN, LOW);
  delay(1000);
}