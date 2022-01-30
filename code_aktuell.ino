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

void halbUmdrehungRechts() { //Quasi 90* Drehung
    outLeft = 200;
    outRight = 0;
    motorAnsteuern();
    delay(900);
    outLeft = 0;
    motorAnsteuern();
}

void halbUmdrehungLinks() { //Quasi 90* Drehung
    outLeft = 0;
    outRight = 200;
    motorAnsteuern();
    delay(900);
    outRight = 0;
    motorAnsteuern();
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
  entfernungMessenVorne(); // bot misst erstmal überall die entfernung
  entfernungMessenLinks();
  entfernungMessenRechts();
  Serial.println("-------"); //print dass man sich auskennt im serial monitor
  if (entfernungVorne < 12) { //bot schaut ob vorne eine wand ist, die maximal 12cm entfernt ist, wenn ja:
    stehenbleiben(); //erstmal stehenbleiben
    if (entfernungLinks > 17) { //schaut ob ein hindernis links vorhanden ist, speicher dann
      hindernisLinks = 1;
    }
    if(entfernungRechts > 17) { //schaut ob ein hindernis rechts vorhanden ist, speicher dann
      hindernisRechts = 1;
    }
/*
    if (2 == (hindernisLinks + hindernisRechts)) {
      stehenbleiben();
      Serial.println("Überall ein Hindernis, bleibe stehen.");
      delay(5000);
    }*/
    if (hindernisLinks = 1) {
      halbUmdrehungRechts();
      Serial.println("Fahre rechts da links hindernis");
    }
    if (hindernisRechts = 1) {
      halbUmdrehungLinks();
      Serial.println("Fahre links da rechts Hindernis");
    }
    if (0 >= (hindernisLinks + hindernisRechts)) {
      Serial.println("Kein Hindernis vorhanden, fährt nach rechts");
      halbUmdrehungRechts();
    }
    /*
    if(1 >= (hindernisLinks + hindernisRechts)) { //schauen ob mindestens ein hindernis vorhanden ist
      if(hindernisLinks = 1) { //wenn links ein hindernis ist wird nach rechts gefahren
        halbUmdrehungRechts();
        Serial.println("Fährt 90° nach rechts");
      } //AB HIER BUG: AUCH WENN KEINS IS WIRD NACH RECHTS GEFAHREN!!!
      if(hindernisRechts = 1) { //wenn links ein hindernis ist wird nach rechts gefahren
        halbUmdrehungLinks();
        Serial.println("Fährt 90° nach links");
      }
      stehenbleiben();
    }
    else {
      halbUmdrehungRechts();
    }*/
    hindernisLinks = 0; //Temporäre Variablen wieder auf 0 setzten für die nächste Kurve
    hindernisRechts = 0; //Temporäre Variablen wieder auf 0 setzten für die nächste Kurve
    fahrenBeide(); //wieder losfahren
  
    // Kurve ende


    /*
    entfernungMessenLinks(); //entfernungen nach der kurve zur sicherheit messen
    entfernungMessenRechts();

    if (entfernungLinks != entfernungRechts) {
      if (entfernungLinks > entfernungRechts) {
        stehenbleiben();
        outRight = 100;
        motorAnsteuern();
        delay(150);
        fahrenBeide();
      }
      if (entfernungRechts > entfernungLinks) {
        stehenbleiben();
        outLeft = 100;
        motorAnsteuern();
        delay(150);
        fahrenBeide();
      }
    }*/

  } 
  delay(500); //Delay dass der ned durchdreht ;)
}
//Code Ende
