#include <Arduino.h>
#include "PCF8575.h"
PCF8575 pcf8575(0x21); //portexpander bus adresse und name 

#define RIGHT_RPWM 5
#define RIGHT_LPWM 6
#define RIGHT_REN 8
#define RIGHT_LEN 9

#define LEFT_RPWM 10
#define LEFT_LPWM 11
#define LEFT_REN 2
#define LEFT_LEN 3

int outLeft;
int outRight;

// Ultraschall vorne/MITTE
#define TRIGGER_VORNE P0 //auf portexpander 
#define ECHO_VORNE 13 
// Ultraschall links
#define TRIGGER_LINKS P1 //auf portexpander 
#define ECHO_LINKS 12
//Ultraschall rechts
#define TRIGGER_RECHTS P3  //auf portexpander
#define ECHO_RECHTS 4

// Ultraschall
long dauerVorne=0; // Dauer Speicher für Ultraschcallsensor vorne
long entfernungVorne=0; // Entfernung Speicher für Ultraschcallsensor vorne
long dauerLinks=0; 
long entfernungLinks=0; 
long dauerRechts=0; 
long entfernungRechts=0;
int entfernungLinksOld; //alte variable wird hier gespeichert
int entfernungRechtsOld;


int umdrehungZeit=840;
int umdrehungSpeed=110;
int istInModus=0;
int sollFahren = 0;


//LED
#define LED_PIN P2


void ledAn() {
  pcf8575.digitalWrite(LED_PIN, LOW); //led wird angeschaltet
}
void ledAus() {
  pcf8575.digitalWrite(LED_PIN, HIGH); //led wird ausgeschaltet
}

// distance
long readDistance(int trigger, int echo) { //main
  pcf8575.digitalWrite(trigger, LOW); //Hier nimmt man die Spannung für kurze Zeit vom Trigger-Pin, damit man später beim Senden des Trigger-Signals ein rauschfreies Signal hat.
  delay(5); // Pause 5 Millisekunden
  pcf8575.digitalWrite(trigger, HIGH); //Jetzt sendet man eine Ultraschallwelle los.
  delay(10); //Dieser „Ton“ erklingt für 10 Millisekunden.
  pcf8575.digitalWrite(trigger, LOW);//Dann wird der „Ton“ abgeschaltet.
  long dauer = pulseIn(echo, HIGH); //Mit dem Befehl „pulseIn“ zählt der Mikrokontroller die Zeit in Mikrosekunden, bis der Schall zum Ultraschallsensor zurückkehrt.
  return (long)((dauer/2) * 0.03432); //Nun berechnet man die Entfernung in Zentimetern. Man teilt zunächst die Zeit durch zwei (Weil man ja nur eine Strecke berechnen möchte und nicht die Strecke hin- und zurück). Den Wert multipliziert man mit der Schallgeschwindigkeit in der Einheit Zentimeter/Mikrosekunde und erhält dann den Wert in Zentimetern.
 }
long readDistancePE(int trigger, int echo) { //main, mit Portexpander
  pcf8575.digitalWrite(trigger, LOW); //Hier nimmt man die Spannung für kurze Zeit vom Trigger-Pin, damit man später beim Senden des Trigger-Signals ein rauschfreies Signal hat.
  delay(5); // Pause 5 Millisekunden
  pcf8575.digitalWrite(trigger, HIGH); //Jetzt sendet man eine Ultraschallwelle los.
  delay(10); //Dieser „Ton“ erklingt für 10 Millisekunden.
  pcf8575.digitalWrite(trigger, LOW);//Dann wird der „Ton“ abgeschaltet.
  long dauer = pulseIn(echo, HIGH); //Mit dem Befehl „pulseIn“ zählt der Mikrokontroller die Zeit in Mikrosekunden, bis der Schall zum Ultraschallsensor zurückkehrt.
  return (long)((dauer/2) * 0.03432); //Nun berechnet man die Entfernung in Zentimetern. Man teilt zunächst die Zeit durch zwei (Weil man ja nur eine Strecke berechnen möchte und nicht die Strecke hin- und zurück). Den Wert multipliziert man mit der Schallgeschwindigkeit in der Einheit Zentimeter/Mikrosekunde und erhält dann den Wert in Zentimetern.
 }
long readDistanceFront() { //front ultraschall
  return readDistancePE(TRIGGER_VORNE, ECHO_VORNE);
 }
long readDistanceLeft() { //left ultraschall
  return readDistancePE(TRIGGER_LINKS, ECHO_LINKS);
 }
long readDistanceRight() { //right ultraschall
  return readDistancePE(TRIGGER_RECHTS, ECHO_RECHTS);
 }

void motorAnsteuern() {
  analogWrite(RIGHT_LPWM,outRight);
  analogWrite(RIGHT_RPWM,0);
  analogWrite(LEFT_LPWM,outLeft);
  analogWrite(LEFT_RPWM,0);
}


void fahrenBeide() {
  outLeft = 100; //Setze Geschwindigkeit links auf 100
  outRight = 100; //Setze Geschwindigkeit rechts auf 100
  motorAnsteuern();
//  Serial.println("fahren beide laut Methode");
}
 
void stehenbleiben() {
  outLeft = 0;
  outRight = 0;
  motorAnsteuern();
}

  
void halbUmdrehungRechts() { //Quasi 90* Drehung nach rechts
    outLeft = umdrehungSpeed;
    outRight = 0;
    motorAnsteuern();
    delay(umdrehungZeit);
    outLeft = 0;
    motorAnsteuern();
    Serial.println("Rechts umdrehung");
}
void halbUmdrehungLinks() { //Quasi 90* Drehung nach links
    outLeft = 0;
    outRight = umdrehungSpeed;
    motorAnsteuern();
    delay(umdrehungZeit);
    outRight = 0;
    motorAnsteuern();
    Serial.println("Links umdrehung");
}



void setup() {
  Serial.begin(9600);
  pinMode(RIGHT_RPWM,OUTPUT);
  pinMode(RIGHT_LPWM,OUTPUT);
  pinMode(RIGHT_LEN,OUTPUT);
  pinMode(RIGHT_REN,OUTPUT);
  digitalWrite(RIGHT_REN,HIGH);
  digitalWrite(RIGHT_LEN,HIGH);

  pinMode(LEFT_RPWM,OUTPUT);
  pinMode(LEFT_LPWM,OUTPUT);
  pinMode(LEFT_LEN,OUTPUT);
  pinMode(LEFT_REN,OUTPUT);
  digitalWrite(LEFT_REN,HIGH);
  digitalWrite(LEFT_LEN,HIGH);

  outLeft = 100;
  outRight = 100;


  // Abstandssensor vorne
  pcf8575.pinMode(TRIGGER_VORNE, OUTPUT); // Trigger-Pin ist ein Ausgang
  pinMode(ECHO_VORNE, INPUT); // Echo-Pin ist ein Eingang
  // Abstandssensor links
  pcf8575.pinMode(TRIGGER_LINKS, OUTPUT); // Trigger-Pin ist ein Ausgang
  pinMode(ECHO_LINKS, INPUT); // Echo-Pin ist ein Eingang
  // Abstandssensor rechts
  pcf8575.pinMode(TRIGGER_RECHTS, OUTPUT); // Trigger-Pin ist ein Ausgang
  pinMode(ECHO_RECHTS, INPUT); // Echo-Pin ist ein Eingang
  //LED
  pcf8575.pinMode(LED_PIN, OUTPUT);



  pcf8575.begin(); //HIER DRUNTER KEIN PORTEXPANDER ZEUG MEHR, HIER WIRD BEGONNEN
}
 
void loop() {
  fahrenBeide();

}
