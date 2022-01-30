#include <Arduino.h>

/*
  MAI Cup Junior Code
  made at 25 Jan 2022
  by Christoph & Emanuel @ MAI Robotics
  Home
  26.01.2022
*/

//Todos: Zeit für Kurve einfügen, Pins einstellen, Links und rechts verkabeln, Besser Verkabeln
// - Pins -
// Motor rechts
#define RIGHT_RPWM 5
#define RIGHT_LPWM 6
#define RIGHT_REN 8
#define RIGHT_LEN 9
//Motor links
#define LEFT_RPWM 10
#define LEFT_LPWM 11
#define LEFT_REN 2
#define LEFT_LEN 3
// Ultraschall vorne
#define TRIGGER_VORNE 12 //Ultraschallsensor vorne Trigger Pin 
#define ECHO_VORNE 13    //Ultraschallsensor vorne Echo Pin 
// Ultraschall links
#define TRIGGER_LINKS 7 
#define ECHO_LINKS A5
//Ultraschall rechts
#define TRIGGER_RECHTS 4 
#define ECHO_RECHTS A4

// - Daten -
int outLeft; 
int outRight;

int logicRight; //Temporärer Speicher bei der Kurve
int logicLeft; //Temporärer Speicher bei der Kurve
int logicRight1; //Temporärer Speicher bei der Kurve
int logicLeft1; //Temporärer Speicher bei der Kurve
int logicRight2; //Temporärer Speicher bei der Kurve
int logicLeft2; //Temporärer Speicher bei der Kurve

long dauerVorne=0; // Dauer Speicher für Ultraschcallsensor vorne
long entfernungVorne=0; // Entfernung Speicher für Ultraschcallsensor vorne

long dauerLinks=0; 
long entfernungLinks=0; 

long dauerRechts=0; 
long entfernungRechts=0;

void motorAnsteuern() {
  analogWrite(RIGHT_LPWM,outRight); //Schreibe Geschwindigkeit auf Pins
  analogWrite(RIGHT_RPWM,0);        //Schreibe Geschwindigkeit auf Pins
  analogWrite(LEFT_LPWM,outLeft);   //Schreibe Geschwindigkeit auf Pins
  analogWrite(LEFT_RPWM,0);         //Schreibe Geschwindigkeit auf Pins
}

/*
 * Motoren starten (beiden fahren)
 */
void fahrenBeide() {
  outLeft = 100; //Setze Geschwindigkeit links auf 100
  outRight = 100; //Setze Geschwindigkeit rechts auf 100
  motorAnsteuern();
  Serial.println("fahren beide laut Methode");
}

/*
 * Motoren stoppen (beide)
 */
void stehenbleiben() {
  outLeft = 0;
  outRight = 0;
  motorAnsteuern();
  Serial.println("stehen beide laut Variable");
}

/*
 * Methode zum Messen der Entfernung
 * @todo: links und rechts wieder einkommentieren
 */
void entfernungMessenVorne() {
  digitalWrite(TRIGGER_VORNE, LOW); //Hier nimmt man die Spannung für kurze Zeit vom Trigger-Pin, damit man später beim Senden des Trigger-Signals ein rauschfreies Signal hat.
  delay(5); // Pause 5 Millisekunden
  digitalWrite(TRIGGER_VORNE, HIGH); //Jetzt sendet man eine Ultraschallwelle los.
  delay(10); //Dieser „Ton“ erklingt für 10 Millisekunden.
  digitalWrite(TRIGGER_VORNE, LOW);//Dann wird der „Ton“ abgeschaltet.
  dauerVorne = pulseIn(ECHO_VORNE, HIGH); //Mit dem Befehl „pulseIn“ zählt der Mikrokontroller die Zeit in Mikrosekunden, bis der Schall zum Ultraschallsensor zurückkehrt.
  entfernungVorne = (long)((dauerVorne/2) * 0.03432); //Nun berechnet man die Entfernung in Zentimetern. Man teilt zunächst die Zeit durch zwei (Weil man ja nur eine Strecke berechnen möchte und nicht die Strecke hin- und zurück). Den Wert multipliziert man mit der Schallgeschwindigkeit in der Einheit Zentimeter/Mikrosekunde und erhält dann den Wert in Zentimetern.
  
  if (entfernungVorne >= 500 || entfernungVorne <= 0) {//Wenn die gemessene Entfernung über 500cm oder unter 0cm liegt,…
    Serial.println("Kein Messwert"); //dann soll der serial monitor ausgeben „Kein Messwert“, weil Messwerte in diesen Bereichen falsch oder ungenau sind.
  }
  else {
    Serial.print(entfernungVorne); //…soll der Wert der Entfernung an den serial monitor hier ausgegeben werden.
    Serial.println(" cm Vorne"); // Hinter dem Wert der Entfernung soll auch am Serial Monitor die Einheit "cm" angegeben werden, danach eine neue Zeile
  }
}

void entfernungMessenLinks() {
  digitalWrite(TRIGGER_LINKS, LOW); //Hier nimmt man die Spannung für kurze Zeit vom Trigger-Pin, damit man später beim Senden des Trigger-Signals ein rauschfreies Signal hat.
  delay(5); // Pause 5 Millisekunden
  digitalWrite(TRIGGER_LINKS, HIGH); //Jetzt sendet man eine Ultraschallwelle los.
  delay(10); //Dieser „Ton“ erklingt für 10 Millisekunden.
  digitalWrite(TRIGGER_LINKS, LOW);//Dann wird der „Ton“ abgeschaltet.
  dauerLinks = pulseIn(ECHO_LINKS, HIGH); //Mit dem Befehl „pulseIn“ zählt der Mikrokontroller die Zeit in Mikrosekunden, bis der Schall zum Ultraschallsensor zurückkehrt.
  entfernungLinks = (long)((dauerLinks/2) * 0.03432); //Nun berechnet man die Entfernung in Zentimetern. Man teilt zunächst die Zeit durch zwei (Weil man ja nur eine Strecke berechnen möchte und nicht die Strecke hin- und zurück). Den Wert multipliziert man mit der Schallgeschwindigkeit in der Einheit Zentimeter/Mikrosekunde und erhält dann den Wert in Zentimetern.
  
  if (entfernungLinks >= 500 || entfernungLinks <= 0) {//Wenn die gemessene Entfernung über 500cm oder unter 0cm liegt,…
    Serial.println("Kein Messwert"); //dann soll der serial monitor ausgeben „Kein Messwert“, weil Messwerte in diesen Bereichen falsch oder ungenau sind.
  }
  else {
    Serial.print(entfernungLinks); //…soll der Wert der Entfernung an den serial monitor hier ausgegeben werden.
    Serial.println(" cm Links"); // Hinter dem Wert der Entfernung soll auch am Serial Monitor die Einheit "cm" angegeben werden, danach eine neue Zeile
  }
}


void entfernungMessenRechts() {
  digitalWrite(TRIGGER_RECHTS, LOW); //Hier nimmt man die Spannung für kurze Zeit vom Trigger-Pin, damit man später beim Senden des Trigger-Signals ein rauschfreies Signal hat.
  delay(5); // Pause 5 Millisekunden
  digitalWrite(TRIGGER_RECHTS, HIGH); //Jetzt sendet man eine Ultraschallwelle los.
  delay(10); //Dieser „Ton“ erklingt für 10 Millisekunden.
  digitalWrite(TRIGGER_RECHTS, LOW);//Dann wird der „Ton“ abgeschaltet.
  dauerRechts = pulseIn(ECHO_RECHTS, HIGH); //Mit dem Befehl „pulseIn“ zählt der Mikrokontroller die Zeit in Mikrosekunden, bis der Schall zum Ultraschallsensor zurückkehrt.
  entfernungRechts = (long)((dauerRechts/2) * 0.03432); //Nun berechnet man die Entfernung in Zentimetern. Man teilt zunächst die Zeit durch zwei (Weil man ja nur eine Strecke berechnen möchte und nicht die Strecke hin- und zurück). Den Wert multipliziert man mit der Schallgeschwindigkeit in der Einheit Zentimeter/Mikrosekunde und erhält dann den Wert in Zentimetern.
  
  if (entfernungRechts >= 500 || entfernungRechts <= 0) {//Wenn die gemessene Entfernung über 500cm oder unter 0cm liegt,…
    Serial.println("Kein Messwert"); //dann soll der serial monitor ausgeben „Kein Messwert“, weil Messwerte in diesen Bereichen falsch oder ungenau sind.
  }
  else {
    Serial.print(entfernungRechts); //…soll der Wert der Entfernung an den serial monitor hier ausgegeben werden.
    Serial.println(" cm Rechts"); // Hinter dem Wert der Entfernung soll auch am Serial Monitor die Einheit "cm" angegeben werden, danach eine neue Zeile
  }
}

void umdrehungZeit() {
    delay(900);                          //HIER ZEIT EINFÜGEN WIE LANG ES DAUERT FÜR EINE KURVE
}

void kursUmdrehungZeit() { //Zeit um wieder auf den Kurs zu kommen
    delay(100);                          //HIER ZEIT EINFÜGEN WIE LANG ES DAUERT FÜR EINE UMDREHUNG
}



// ------------------------------------------------------------------------------------
// -                                Ende der Methoden                                 -
// ------------------------------------------------------------------------------------



// - Erster Start -

void setup() {
  Serial.begin(9600); //Starte den Serial Monitor
  // Motor rechts
  pinMode(RIGHT_RPWM,OUTPUT); //Pin-Modus setzen --> Pulsweitenmodulation
  pinMode(RIGHT_LPWM,OUTPUT); //Pin-Modus setzen --> Pulsweitenmodulation
  pinMode(RIGHT_LEN,OUTPUT);  //Pin-Modus setzen --> Pulsweitenmodulation
  pinMode(RIGHT_REN,OUTPUT);  //Pin-Modus setzen --> Pulsweitenmodulation
  digitalWrite(RIGHT_REN,HIGH); //Pin beschreiben
  digitalWrite(RIGHT_LEN,HIGH); //Pin beschreiben
  // Motor links
  pinMode(LEFT_RPWM,OUTPUT); //Pin-Modus setzen --> Pulsweitenmodulation
  pinMode(LEFT_LPWM,OUTPUT); //Pin-Modus setzen --> Pulsweitenmodulation
  pinMode(LEFT_LEN,OUTPUT);  //Pin-Modus setzen --> Pulsweitenmodulation
  pinMode(LEFT_REN,OUTPUT);  //Pin-Modus setzen --> Pulsweitenmodulation
  digitalWrite(LEFT_REN,HIGH); //Pin beschreiben
  digitalWrite(LEFT_LEN,HIGH); //Pin beschreiben
  // Abstandssensor vorne
  pinMode(TRIGGER_VORNE, OUTPUT); // Trigger-Pin ist ein Ausgang
  pinMode(ECHO_VORNE, INPUT); // Echo-Pin ist ein Eingang
  // Abstandssensor links
  pinMode(TRIGGER_LINKS, OUTPUT); // Trigger-Pin ist ein Ausgang
  pinMode(ECHO_LINKS, INPUT); // Echo-Pin ist ein Eingang
  // Abstandssensor rechts
  pinMode(TRIGGER_RECHTS, OUTPUT); // Trigger-Pin ist ein Ausgang
  pinMode(ECHO_RECHTS, INPUT); // Echo-Pin ist ein Eingang

  
  Serial.println("--- Pins gesetzt");
  fahrenBeide(); //Bot startet das Fahren
  Serial.println("--- Im Setup Fahren gestartet");
  delay(3000);
}


/*
 * Main Loop
 */
void loop() {
  entfernungMessenVorne();
  entfernungMessenLinks();
  entfernungMessenRechts();
  Serial.println("-------");
  if (entfernungVorne < 12) { //Check ob Hindernis, danach Kurve
    stehenbleiben(); //erstmal stehenbleiben
    Serial.println("--- Stehen bleiben laut Check vorne kleiner als 12cm");
    if (entfernungLinks > 30) { //schauen ob entfernung links kleiner als 30cm ist, speichert dann ab ob hier ein hindernis ist oder nicht (nur temporär)
      logicRight = 1;
    }
    if(entfernungRechts > 30) { //schauen ob entfernung rechts kleiner als 30cm ist, speichert dann ab ob hier ein hindernis ist oder nicht
      logicLeft = 1;
    }
    if(2 >= (logicRight + logicLeft)) { //schauen ob ein Hindernis vorhanden ist
      if(logicRight = 1) { //wenn rechts ein hindernis ist wird nach links gefahren
        outRight = 200;
        Serial.println("--- Right 100 laut if schleife");
      }
      else { //wenn links ein hindernis ist wird nach rechts gefahren
        outLeft = 200;
        Serial.println("--- Left 100 laut if schleife");
      }
      motorAnsteuern();
      umdrehungZeit();
      stehenbleiben();
    }
    logicLeft = 0; //Temporäre Variablen wieder auf 0 setzten für die nächste Kurve
    logicRight = 0; //Temporäre Variablen wieder auf 0 setzten für die nächste Kurve
    Serial.println("--- logicRight 0 beide laut if schleife ende (vor kurve ende)");
    fahrenBeide(); //wieder losfahren
    Serial.println("--- Fahren laut schleife ende (vor kurve ende)");
  
    // Kurve ende
    entfernungMessenLinks();
    entfernungMessenRechts();
    Serial.println("--- Kurve ende beginnt");
    if (entfernungLinks <= 10) {
      logicRight1 = 1; //Logicright1 -> Links
    }
    if (entfernungRechts <= 10) {
      logicLeft2 = 1;
    }
    if (logicLeft = 1) {
      outLeft = 200;
      motorAnsteuern();
      kursUmdrehungZeit();
      outLeft = 100;
      outRight = 100;
      motorAnsteuern();
      logicLeft = 0;
      logicRight = 0;
    }
    else {
      if (logicRight = 1) {
        outRight = 200;
        motorAnsteuern();
        kursUmdrehungZeit();
        outLeft = 100;
        outRight = 100;
        motorAnsteuern();
        logicLeft = 0;
        logicRight = 0;
      }
    }
  } 
  delay(500); //Delay dass der ned durchdreht ;)
}
//Code Ende
