#include <Arduino.h>
#include "PCF8575.h"

PCF8575 pcf8575(0x21);

#define SENSOR_S0 P4
#define SENSOR_S1 P5
#define SENSOR_S2 P6
#define SENSOR_S3 P7
#define SENSOR_OUT 7 
int frequency = 0;
int redWert;
int grunWert;
int blueWert;

int readRedColor() {
  pcf8575.digitalWrite(SENSOR_S2, LOW);
  pcf8575.digitalWrite(SENSOR_S3, LOW);
  return  pulseIn(SENSOR_OUT, LOW);
}

int readGreenColor() {
  pcf8575.digitalWrite(SENSOR_S2, HIGH);
  pcf8575.digitalWrite(SENSOR_S3, HIGH);
  return pulseIn(SENSOR_OUT, LOW);
}

int readBlueColor() {
  pcf8575.digitalWrite(SENSOR_S2, LOW);
  pcf8575.digitalWrite(SENSOR_S3, HIGH);
  return pulseIn(SENSOR_OUT, LOW);
}

void setup() {
    Serial.begin(9600);
    pcf8575.pinMode(SENSOR_S0, OUTPUT); //portexpander
  pcf8575.pinMode(SENSOR_S1, OUTPUT); //portexpander
  pcf8575.pinMode(SENSOR_S2, OUTPUT); //portexpander
  pcf8575.pinMode(SENSOR_S3, OUTPUT); //portexpander
  pinMode(SENSOR_OUT, INPUT);
  pcf8575.digitalWrite(SENSOR_S0, HIGH); //portexpander
  pcf8575.digitalWrite(SENSOR_S1, LOW);
  pcf8575.begin(); 
  Serial.println("----- INFO: Pins gesetzt");


}

void loop() {
  if(readGreenColor() < 30) {
    Serial.println("Zielzone_erkannt");
  } else {
    Serial.println("Keine_Zielzone_erkannt");
  }
  delay(100)
}