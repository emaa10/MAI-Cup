#include <Arduino.h>
#include "PCF8575.h"

PCF8575 pcf8575(0x21); //portexpander bus adresse und name 

#define TRIGGER_LINKS P1 //auf portexpander 
#define ECHO_LINKS 12

// Ultraschall
long dauerVorne=0; // Dauer Speicher für Ultraschcallsensor vorne
long entfernungVorne=0; // Entfernung Speicher für Ultraschcallsensor vorne

long dauerLinks=0; 
long entfernungLinks=0; 

long dauerRechts=0; 
long entfernungRechts=0;

int entfernungLinksOld; //alte variable wird hier gespeichert
int entfernungRechtsOld;

void entfernungMessenLinks() {
  entfernungLinksOld = entfernungLinks;
  pcf8575.digitalWrite(TRIGGER_LINKS, LOW); //Hier nimmt man die Spannung für kurze Zeit vom Trigger-Pin, damit man später beim Senden des Trigger-Signals ein rauschfreies Signal hat.
  delay(5); // Pause 5 Millisekunden
  pcf8575.digitalWrite(TRIGGER_LINKS, HIGH); //Jetzt sendet man eine Ultraschallwelle los.
  delay(10); //Dieser „Ton“ erklingt für 10 Millisekunden.
  pcf8575.digitalWrite(TRIGGER_LINKS, LOW);//Dann wird der „Ton“ abgeschaltet.
  dauerLinks = pulseIn(ECHO_LINKS, HIGH); //Mit dem Befehl „pulseIn“ zählt der Mikrokontroller die Zeit in Mikrosekunden, bis der Schall zum Ultraschallsensor zurückkehrt.
  entfernungLinks = (long)((dauerLinks/2) * 0.03432); //Nun berechnet man die Entfernung in Zentimetern. Man teilt zunächst die Zeit durch zwei (Weil man ja nur eine Strecke berechnen möchte und nicht die Strecke hin- und zurück). Den Wert multipliziert man mit der Schallgeschwindigkeit in der Einheit Zentimeter/Mikrosekunde und erhält dann den Wert in Zentimetern.
  
  if (entfernungLinks >= 500 || entfernungLinks <= 0) {//Wenn die gemessene Entfernung über 500cm oder unter 0cm liegt,…
    Serial.print("Links: ");
    Serial.print(entfernungLinks); //dann soll der serial monitor ausgeben „Kein Messwert“, weil Messwerte in diesen Bereichen falsch oder ungenau sind.
  }
  else {
    Serial.print("Links: ");
    Serial.print(entfernungLinks); //…soll der Wert der Entfernung an den serial monitor hier ausgegeben werden.
  //  Serial.print(" cm Links"); // Hinter dem Wert der Entfernung soll auch am Serial Monitor die Einheit "cm" angegeben werden, danach eine neue Zeile
  }
  Serial.print("cm   ");
  //Serial.println(" Alte Entfernung links");
}

void loop() {
  entfernungMessenLinks();
}

void setup() {
    pcf8575.begin();
    Serial.begin(9600);
    Serial.print("amogus");
}