#include <Arduino.h>
#include "PCF8575.h"
PCF8575 pcf8575(0x21); //portexpander bus adresse und name 
/*
  MAI Cup Junior Code
  Micronova Edition
  made at 25 Jan 2022
  by Christoph & Emanuel @ MAI Robotics
  Home
  26.01.2022
*/
// - Pins -
//motor
#define RIGHT_RPWM 5
#define RIGHT_LPWM 6
#define RIGHT_REN 8
#define RIGHT_LEN 9

#define LEFT_RPWM 10
#define LEFT_LPWM 11
#define LEFT_REN 2
#define LEFT_LEN 3
// Ultraschall vorne/MITTE
#define TRIGGER_VORNE P0 //auf portexpander 
#define ECHO_VORNE 13 
// Ultraschall links
#define TRIGGER_LINKS P1 //auf portexpander 
#define ECHO_LINKS 12
//Ultraschall rechts
#define TRIGGER_RECHTS P3  //auf portexpander
#define ECHO_RECHTS 4
//Hall Sensor
//#define HALL_SENSOR A0          //analog output (optional)
#define HALL_SENSOR_D A2        // digital output (benutzt zum auslesen ob magnet oder nd)
int hallValAlt;
int hallValAlt2;
//Infrarot Sensor
#define IR_LEFT A3 // connect ir sensor to arduino pin 2 (left one)
#define IR_RIGHT A1
#define IR_MIDDLE A0
//LED
#define LED_PIN P2
//farbsensor
#define SENSOR_S0 P4
#define SENSOR_S1 P5
#define SENSOR_S2 P6
#define SENSOR_S3 P7
#define SENSOR_OUT 7 //einziger pin der nicht auf portexpander sein muss
// - Daten -
//motor
int outLeft; 
int outRight;
// alte trash logic dinger
int logicRight; //Temporärer Speicher bei der Kurve
int logicLeft; //Temporärer Speicher bei der Kurve
int logicRight1; //Temporärer Speicher bei der Kurve
int logicLeft1; //Temporärer Speicher bei der Kurve
int logicRight2; //Temporärer Speicher bei der Kurve
int logicLeft2; //Temporärer Speicher bei der Kurve
// neue logic checks
int hindernisLinks;
int hindernisRechts;
//Hall Sensor
int Hall_Val1=0,Hall_Val2=0;
enum HallPosition {LINKS, RECHTS};
HallPosition magnetPosition = RECHTS;                                                        // MAGNET CONFIG HIER
// Ultraschall
long dauerVorne=0; // Dauer Speicher für Ultraschcallsensor vorne
long entfernungVorne=0; // Entfernung Speicher für Ultraschcallsensor vorne
long dauerLinks=0; 
long entfernungLinks=0; 
long dauerRechts=0; 
long entfernungRechts=0;
int entfernungLinksOld; //alte variable wird hier gespeichert
int entfernungRechtsOld;
//technik
int durchgangCounter=0;
unsigned long previousMillis = 0;
#define SPEEDSYNCINTERVAL 300
int umdrehungZeit=840;
int umdrehungSpeed=110;
//farbsensor
int frequency = 0;
int redWert;
int grunWert;
int blueWert;
//farbsensor API
int zielzoneMinWertRed = 100;     //zielzone geht von minwert bis maxwert
//int zielzoneMinWertGreen -->   Existiert nicht weil zu ungenau, könnte auch Boden sein!!
int zielzoneMinWertBlue = 85;

int zielzoneMaxWertRed = 150; 
//int zielzoneMaxWertGreen -->   Existiert nicht weil zu ungenau, könnte auch Boden sein!!
int zielzoneMaxWertBlue = 110;

int lineMinWertBlue = 1; 
int lineMinWertGreen = 1;
int lineMinWertRed = 1;

int lineMaxWertBlue = 2;
int lineMaxWertGreen = 2;
int lineMaxWertRed = 2;

int lineTempVarRed;
int lineTempVarGreen;
int lineTempVarBlue;

int istInModus=0;

int oldWertLineRight;
int oldWertLineLeft;

int sollFahren = 0;

 //---------------------------------//



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

// - Methoden -
void motorAnsteuern() {
  analogWrite(RIGHT_LPWM,outRight); //Schreibe Geschwindigkeit auf Pins
  analogWrite(RIGHT_RPWM,0);        //Schreibe Geschwindigkeit auf Pins
  analogWrite(LEFT_LPWM,outLeft);   //Schreibe Geschwindigkeit auf Pins
  analogWrite(LEFT_RPWM,0);         //Schreibe Geschwindigkeit auf Pins
}
void motorAnsteuernGeradeausLauf() {
  if(outLeft >= 80 || outRight >= 80) {
    outLeft -= 30;
    outRight -= 30;
  }
  motorAnsteuern();
}

/*
 * Motoren starten (beiden fahren)
 */
void fahrenBeide() {
  outLeft = 50; //Setze Geschwindigkeit links auf 100
  outRight = 50; //Setze Geschwindigkeit rechts auf 100
  motorAnsteuern();
//  Serial.println("fahren beide laut Methode");
}
/*
 * Motoren stoppen (beide)
 */
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











// --------------------------------------------------------











void setup() {
  Serial.begin(9600); //Starte den Serial Monitor

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


  ///////////////////////////////////////////////////////////////////////////////////////////////
  pcf8575.begin(); //HIER DRUNTER KEIN PORTEXPANDER ZEUG MEHR, HIER WIRD BEGONNEN
  ////////////////////////////////////////////////////////////////////////////////////////////////

}

void loop() {
  ledAus();
  if(readDistanceFront <= 20) {
    fahrenBeide;
    delay(5000);
    while(readDistanceFront >= 20)
        delay(20);
        sollFahren++;
    } 
    halbUmdrehungLinks();
    halbUmdrehungLinks();
    fahrenBeide();
    delay(sollFahren * 20);
    halbUmdrehungRechts;
    halbUmdrehungRechts;
    stehenbleiben;
  
}