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

#define RIGHT_RPWM 5
#define RIGHT_LPWM 6
#define RIGHT_REN 8
#define RIGHT_LEN 9

#define LEFT_RPWM 10
#define LEFT_LPWM 11
#define LEFT_REN 2
#define LEFT_LEN 3

// - Daten -
// !!! PINS STIMMEN NOCH NICHT!!! (nur vorne)

/*
Vorne Echo: 13
Links: A5
Rechts: A4
*/
int outLeft; 
int outRight;
int logicright=0; //Temporärer Speicher bei der Kurve
int logicleft=0; //Temporärer Speicher bei der Kurve
int logicright1; //Temporärer Speicher bei der Kurve
int logicleft1; //Temporärer Speicher bei der Kurve
int logicright2; //Temporärer Speicher bei der Kurve
int logicleft2; //Temporärer Speicher bei der Kurve
#define TRIGGER_VORNE 12 //Ultraschallsensor vorne Trigger Pin 
#define ECHO_VORNE 13    //Ultraschallsensor vorne Echo Pin 
long dauerVorne=0; // Dauer Speicher für Ultraschcallsensor vorne
long entfernungVorne=0; // Entfernung Speicher für Ultraschcallsensor vorne
#define TRIGGER_LINKS 7 
#define ECHO_LINKS A5
long dauerLinks=0; 
long entfernungLinks=0; 
#define TRIGGER_RECHTS 4 
#define ECHO_RECHTS A4
long dauerRechts=0; 
long entfernungRechts=0;

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

  fahrenBeide(); //Bot startet das Fahren
}

/*
 * Motoren starten (beiden fahren)
 */
void fahrenBeide() {
  Serial.println("fahren beide");
  outLeft = 100; //Linker Motor auf Geschwindigkeitsgrad "100" --> maximal 255
  outRight = 100; //Rechter Motor auf Geschwindigkeitsgrad "100" --> maximal 255
}

/*
 * Motoren stoppen (beide)
 */
void stehenbleiben() {
  Serial.println("stehen beide");
  outLeft = 0; //Linker Motor auf Geschwindigkeitsgrad "0"
  outRight = 0; //Rechter Motor auf Geschwindigkeitsgrad "0"
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
    delay(100);                          //HIER ZEIT EINFÜGEN WIE LANG ES DAUERT FÜR EINE KURVE
}

void kursUmdrehungZeit() { //Zeit um wieder auf den Kurs zu kommen
    delay(100);                          //HIER ZEIT EINFÜGEN WIE LANG ES DAUERT FÜR EINE UMDREHUNG
}

// ------------------------------------------------------------------------------------
// -                                Ende der Methoden                                 -
// ------------------------------------------------------------------------------------



/*
 * Main Loop
 */
void loop() {
  delay(1000);
  entfernungMessenVorne();
  entfernungMessenLinks();
  entfernungMessenRechts();

  if (entfernungVorne < 12) { //Check ob Hindernis, danach Kurve
      stehenbleiben(); //erstmal stehenbleiben
        if (entfernungLinks > 30); { //schauen ob entfernung links kleiner als 30cm ist, speichert dann ab ob hier ein hindernis ist oder nicht (nur temporär)
            logicright = 1;
            if(entfernungRechts > 30); { //schauen ob entfernung rechts kleiner als 30cm ist, speichert dann ab ob hier ein hindernis ist oder nicht
                logicleft = 1;
                if(2 >= (logicright + logicleft)); { //schauen ob ein hindernis vorhanden ist
                    if(logicright = 1) { //wenn rechts ein hindernis ist wird nach links gefahren
                        outRight = 100;
                    }
                    
                    else { //wenn links ein hindernis ist wird nach rechts gefahren
                        outLeft = 100;
                    }
                    
                    umdrehungZeit();
                }
            }

        } 
  logicleft = 0; //Temporäre Variablen wieder auf 0 setzten für die nächste Kurve
  logicright = 0; //Temporäre Variablen wieder auf 0 setzten für die nächste Kurve
  fahrenBeide(); //wieder losfahren
  
  // Kurve ende
  entfernungMessenLinks();
  entfernungMessenRechts();
  if (entfernungLinks <= 10) {
  logicright1 = 1; //Logicright1 -> Links
  }
  if (entfernungRechts <= 10) {
      logicleft2 = 1;
  }
  if (logicleft = 1) {
      outLeft = 200;
      kursUmdrehungZeit();
      outLeft = 100;
      outRight = 100;
      logicleft = 0;
      logicright = 0;
  }
    else {
        if (logicright = 1) {
            outRight = 200;
            kursUmdrehungZeit();
            outLeft = 100;
            outRight = 100;
            logicleft = 0;
            logicright = 0;
      }
    }
  } 
}
//Code Ende