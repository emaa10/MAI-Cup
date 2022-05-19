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
#define TRIGGER_VORNE P0 //auf portexpander 
#define ECHO_VORNE 13 
// Ultraschall links
#define TRIGGER_LINKS P1 //auf portexpander 
#define ECHO_LINKS 12
//Ultraschall rechts
#define TRIGGER_RECHTS P3 
#define ECHO_RECHTS 4
//Hall Sensor
//#define HALL_SENSOR A0          //analog output (optional)
#define HALL_SENSOR_D A2        // digital output (benutzt zum auslesen ob magnet oder nd)
int hallValAlt;
//Infrarot Sensor
#define IR_LEFT A3 // connect ir sensor to arduino pin 2 (left one)
#define IR_RIGHT A1
#define IR_MIDDLE A0
//LED
#define LED_PIN P2


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
int linksRechtsCount=0;
int linieaktiv=0;
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
  int statusSensorLeft = digitalRead(IR_LEFT);
  if(statusSensorLeft == 1) { // diese abfrage kann man später auch noch verwenden
    Serial.print("Linie");
    Serial.print("   ");
  } else {
    Serial.print("Boden");
    Serial.print("   ");
  }
}

void linieMitte() {
  int statusSensorMiddle = digitalRead(IR_MIDDLE);
  if(statusSensorMiddle == 1) { // diese abfrage kann man später auch noch verwenden
    Serial.print("Linie");
    Serial.print("   ");
  } else {
    Serial.print("Boden");
    Serial.print("   ");
  }
}

void linieRechts() {
  int statusSensorRight = digitalRead(IR_RIGHT);
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

void linienInformationen() { //liest die aktuellen informationen ab
  int statusSensorLeft = digitalRead(IR_LEFT);
  int statusSensorMiddle = digitalRead(IR_MIDDLE);
  int statusSensorRight = digitalRead(IR_RIGHT);
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
  pinMode(IR_LEFT, INPUT); // sensor pin INPUT
  pinMode(IR_RIGHT, INPUT); // sensor pin INPUT
  pinMode(IR_MIDDLE, INPUT);
  // Hall Sensor
  pinMode(HALL_SENSOR_D,INPUT);

  
  Serial.println("----- INFO: Pins gesetzt");
  fahrenBeide(); //Bot startet das Fahren
  Serial.println("----- INFO: Im Setup Fahren gestartet");
  delay(1500); //Delay dass nicht direkt irgendwelche Hindernisse erkannt werden
}

void loop() {
  int statusSensorLeft = digitalRead(IR_LEFT);
  int statusSensorMiddle = digitalRead(IR_MIDDLE);
  int statusSensorRight = digitalRead(IR_RIGHT);
  if(statusSensorMiddle == 1) {
    linieaktiv = 1;
    outLeft = 100;
    outRight = 100;
    motorAnsteuern();
  }
  if(statusSensorMiddle == 0 && linieaktiv == 1) {
    linksRechtsCount++;
    int statusSensorLeft = digitalRead(IR_LEFT);
    int statusSensorMiddle = digitalRead(IR_MIDDLE);
    int statusSensorRight = digitalRead(IR_RIGHT);
    if(linksRechtsCount % 2) {
        outRight = 100;
        outLeft = 0;
        motorAnsteuern();
        break;
    } else {
        outLeft = 100;
        outRight = 0;
        motorAnsteuern();
        break;
    }
  }
}  