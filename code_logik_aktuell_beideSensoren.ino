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
// Ultraschall vorne/MITTE
#define TRIGGER_VORNE 12 //Ultraschallsensor vorne Trigger Pin 
#define ECHO_VORNE 13    //Ultraschallsensor vorne Echo Pin 
// Ultraschall links
#define TRIGGER_LINKS 7 
#define ECHO_LINKS A5
//Ultraschall rechts
#define TRIGGER_RECHTS 4 
#define ECHO_RECHTS A4
//Hall Sensor
#define HALL_SENSOR A0          //analog output (optional)
#define HALL_SENSOR_D A2        // digital output (benutzt zum auslesen ob magnet oder nd)
//Infrarot Sensor
int ir_left = A3; // connect ir sensor to arduino pin 2 (left one)
int ir_right = A1;


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
//Hall Sensor
int Hall_Val1=0,Hall_Val2=0;
// Ultraschall
long dauerVorne=0; // Dauer Speicher für Ultraschcallsensor vorne
long entfernungVorne=0; // Entfernung Speicher für Ultraschcallsensor vorne

long dauerLinks=0; 
long entfernungLinks=0; 

long dauerRechts=0; 
long entfernungRechts=0;

int entfernungLinksOld; //alte variable wird hier gespeichert
int entfernungRechtsOld;

// - Methoden -

void infoSerial() { //existiert nur für einen überblick über die ausgaben
  //Serial.println("41 cm   41 cm   5 cm   22 cm   23 cm   Nein   Nein   Nein");
  //Serial.println("Left  - Leftold-Vorne - R neu - R old  left - right - magnet      left/right -> Linie");
}

void motorAnsteuern() {
  analogWrite(RIGHT_LPWM,outRight); //Schreibe Geschwindigkeit auf Pins
  analogWrite(RIGHT_RPWM,0);        //Schreibe Geschwindigkeit auf Pins9
  analogWrite(LEFT_LPWM,outLeft);   //Schreibe Geschwindigkeit auf Pins
  analogWrite(LEFT_RPWM,0);         //Schreibe Geschwindigkeit auf Pins
}

void magnetLesen() {
  Hall_Val2=digitalRead(HALL_SENSOR_D);
  if(Hall_Val2 == 0) {
    Serial.print("Ja"); // das hier wird nur zum loggen ausgeführt, man kann danach die hall_val2 trz abfragen für eine andere aktion
  } else {
    Serial.print("Nein");
  }
  Serial.println("   ");
}

void linieLinks() {
  int statusSensorLeft = digitalRead(ir_left);
  if(statusSensorLeft == 1) { // diese abfrage kann man später auch noch verwenden
    Serial.print("Linie");
    Serial.print("   ");
  } else {
    Serial.print("Boden");
    Serial.print("   ");
  }
}

void linieRechts() {
  int statusSensorRight = digitalRead(ir_right);
  if(statusSensorRight == 1) {
    Serial.print("Linie");
    Serial.print("   ");
  } else
    Serial.print("Boden");
    Serial.print("   ");
}

/*
 * Motoren starten (beiden fahren)
 */
void fahrenBeide() {
  outLeft = 105; //Setze Geschwindigkeit links auf 100
  outRight = 110; //Setze Geschwindigkeit rechts auf 100
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
    Serial.print("o/A"); //dann soll der serial monitor ausgeben „Kein Messwert“, weil Messwerte in diesen Bereichen falsch oder ungenau sind.
  }
  else {
    Serial.print(entfernungVorne); //…soll der Wert der Entfernung an den serial monitor hier ausgegeben werden.
  //  Serial.println(" cm Vorne"); // Hinter dem Wert der Entfernung soll auch am Serial Monitor die Einheit "cm" angegeben werden, danach eine neue Zeile
  }
  Serial.print(" cm   ");
}

void entfernungMessenLinks() {
  entfernungLinksOld = entfernungLinks;
  digitalWrite(TRIGGER_LINKS, LOW); //Hier nimmt man die Spannung für kurze Zeit vom Trigger-Pin, damit man später beim Senden des Trigger-Signals ein rauschfreies Signal hat.
  delay(5); // Pause 5 Millisekunden
  digitalWrite(TRIGGER_LINKS, HIGH); //Jetzt sendet man eine Ultraschallwelle los.
  delay(10); //Dieser „Ton“ erklingt für 10 Millisekunden.
  digitalWrite(TRIGGER_LINKS, LOW);//Dann wird der „Ton“ abgeschaltet.
  dauerLinks = pulseIn(ECHO_LINKS, HIGH); //Mit dem Befehl „pulseIn“ zählt der Mikrokontroller die Zeit in Mikrosekunden, bis der Schall zum Ultraschallsensor zurückkehrt.
  entfernungLinks = (long)((dauerLinks/2) * 0.03432); //Nun berechnet man die Entfernung in Zentimetern. Man teilt zunächst die Zeit durch zwei (Weil man ja nur eine Strecke berechnen möchte und nicht die Strecke hin- und zurück). Den Wert multipliziert man mit der Schallgeschwindigkeit in der Einheit Zentimeter/Mikrosekunde und erhält dann den Wert in Zentimetern.
  
  if (entfernungLinks >= 500 || entfernungLinks <= 0) {//Wenn die gemessene Entfernung über 500cm oder unter 0cm liegt,…
    Serial.print("o/A"); //dann soll der serial monitor ausgeben „Kein Messwert“, weil Messwerte in diesen Bereichen falsch oder ungenau sind.
  }
  else {
    Serial.print(entfernungLinks); //…soll der Wert der Entfernung an den serial monitor hier ausgegeben werden.
  //  Serial.print(" cm Links"); // Hinter dem Wert der Entfernung soll auch am Serial Monitor die Einheit "cm" angegeben werden, danach eine neue Zeile
  }
  Serial.print(" cm   ");
  Serial.print(entfernungLinksOld);
  Serial.print(" cm   ");
  //Serial.println(" Alte Entfernung links");
}


void entfernungMessenRechts() {
  entfernungRechtsOld = entfernungRechts;
  digitalWrite(TRIGGER_RECHTS, LOW); //Hier nimmt man die Spannung für kurze Zeit vom Trigger-Pin, damit man später beim Senden des Trigger-Signals ein rauschfreies Signal hat.
  delay(5); // Pause 5 Millisekunden
  digitalWrite(TRIGGER_RECHTS, HIGH); //Jetzt sendet man eine Ultraschallwelle los.
  delay(10); //Dieser „Ton“ erklingt für 10 Millisekunden.
  digitalWrite(TRIGGER_RECHTS, LOW);//Dann wird der „Ton“ abgeschaltet.
  dauerRechts = pulseIn(ECHO_RECHTS, HIGH); //Mit dem Befehl „pulseIn“ zählt der Mikrokontroller die Zeit in Mikrosekunden, bis der Schall zum Ultraschallsensor zurückkehrt.
  entfernungRechts = (long)((dauerRechts/2) * 0.03432); //Nun berechnet man die Entfernung in Zentimetern. Man teilt zunächst die Zeit durch zwei (Weil man ja nur eine Strecke berechnen möchte und nicht die Strecke hin- und zurück). Den Wert multipliziert man mit der Schallgeschwindigkeit in der Einheit Zentimeter/Mikrosekunde und erhält dann den Wert in Zentimetern.
  
  if (entfernungRechts >= 500 || entfernungRechts <= 0) {//Wenn die gemessene Entfernung über 500cm oder unter 0cm liegt,…
    Serial.print("o/A"); //dann soll der serial monitor ausgeben „Kein Messwert“, weil Messwerte in diesen Bereichen falsch oder ungenau sind.
  }
  else {
    Serial.print(entfernungRechts); //…soll der Wert der Entfernung an den serial monitor hier ausgegeben werden.
  //  Serial.println(" cm Rechts"); // Hinter dem Wert der Entfernung soll auch am Serial Monitor die Einheit "cm" angegeben werden, danach eine neue Zeile
  }
  Serial.print(" cm   ");
  Serial.print(entfernungRechtsOld);
  Serial.print(" cm   ");
}

void umdrehungZeit() {
    delay(830);                          //HIER ZEIT EINFÜGEN WIE LANG ES DAUERT FÜR EINE KURVE
}
// outdated
void kursUmdrehungZeit() { //Zeit um wieder auf den Kurs zu kommen
    delay(100);                          //HIER ZEIT EINFÜGEN WIE LANG ES DAUERT FÜR EINE UMDREHUNG
}

void halbUmdrehungRechts() { //Quasi 90* Drehung nach rechts
    outLeft = 200;
    outRight = 0;
    motorAnsteuern();
    umdrehungZeit();
    outLeft = 0;
    motorAnsteuern();
}

void halbUmdrehungLinks() { //Quasi 90* Drehung nach links
    outLeft = 0;
    outRight = 200;
    motorAnsteuern();
    umdrehungZeit();
    outRight = 0;
    motorAnsteuern();
}

void kurzerAusgleichNachLinks() {
  outRight = 200;
  motorAnsteuern();
  delay(250);
  outRight = 110;
  motorAnsteuern();
}

void kurzerAusgleichNachRechts() {
  outLeft = 200;
  motorAnsteuern();
  delay(250);
  outLeft = 105;
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
  // Infrarotsensoren
  pinMode(ir_left, INPUT); // sensor pin INPUT
  pinMode(ir_right, INPUT); // sensor pin INPUT
  // Hall Sensor
  pinMode(HALL_SENSOR_D,INPUT);

  
  Serial.println("----- INFO: Pins gesetzt");
  fahrenBeide(); //Bot startet das Fahren
  Serial.println("----- INFO: Im Setup Fahren gestartet");
  delay(1500); //Delay dass nicht direkt irgendwelche Hindernisse erkannt werden
}


/*
 * Main Loop
 */
void loop() {
  entfernungMessenLinks(); // er misst durchgehend die entfernung nach vorne
  entfernungMessenVorne(); //entfernung links und rechts messen wenn vorne nh wand is
  entfernungMessenRechts();
  linieLinks();
  linieRechts();
  magnetLesen();
  if (entfernungVorne <= 23) { //wenn vorne eine wand ist dann fängt er an links und rechts zu messen
    stehenbleiben(); //direkt stehenbleiben
    if (entfernungLinks <= 23) { //wenn links eine wand ist wird hindernisLinks auf 1 gesetzt (wenn links weniger als 0 cm entfernt ist auch, also bei einem messfehler)
      hindernisLinks = 1;
    }
    if (entfernungRechts <= 23) { //wenn rechts eine wand ist wird hindernisRechts auf 1 gesetzt
      hindernisRechts = 1;
    }


    //Abfrage start 
    if (2 == hindernisLinks + hindernisRechts) { //wenn 2 hindernisse vorhanden sind --> stehen bleiben (noch kein richtiger code hier gefunden)
      stehenbleiben();
      Serial.println("----- INFO: Stehen geblieben, da 2 Hindernisse vorhanden sind");
    }
    else if (hindernisLinks == 1) { // wenn links ein hindernis ist, fährt er wieder los und gibt eine ausgabe (als erstes mal zum testen)
      halbUmdrehungRechts();
      fahrenBeide();
      //Serial.println("----- INFO: Links hindernis fährt also nach rechts -----");
    }
    else if (hindernisRechts == 1) { 
      halbUmdrehungLinks();
      fahrenBeide();
      //Serial.println("----- INFO: Rechts hindernis fährt also nach links -----");
    }
    else if (0 == hindernisLinks + hindernisRechts) { //bei keinem hindernis und nur vorne fährt er halt rechts
      //Serial.println("----- INFO: Kein Hindernis links/rechts --> fährt nach rechts");
      halbUmdrehungRechts();
      fahrenBeide();
    }
  }

  //gerade fahren skript beginn

  if (entfernungRechtsOld > entfernungRechts) { //er speichert alle 200ms die alte und die neue entfernung und vergleicht beide variablen dann. wenn rechts vorher weiter weg war, nähert er sich nach rechts an und fährt nun nach links (nur kurz). umgekehrt halt genauso
    kurzerAusgleichNachLinks();
    //Serial.println("----- INFO: Rechts war davor weiter weg, daher fährt er kurz nach links");
  }
  if (entfernungRechts > entfernungRechtsOld) {
    kurzerAusgleichNachRechts();
    //Serial.println("----- INFO: Links war davor weiter weg, daher fährt er kurz nach rechts");
  }
  int statusSensorLeft = digitalRead(ir_left);
  int statusSensorRight = digitalRead(ir_Right);
  //linienabfrage:
  if (statusSensorLeft == 1) { //wenn sensor links auf linie ist
    halbUmdrehungLinks(); //soll er nach links fahren weil er ja nur die abbiegung wahrnimmt
  }
  if (statusSensorRight == 1) {
    halbUmdrehungRechts();
  }

  infoSerial();
  //delay(200); //zum Testen, könnten wir gut dalassen
  hindernisLinks = 0;
  hindernisRechts = 0;
}
//Code Ende
// test vscode 3