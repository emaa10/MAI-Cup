#include <Arduino.h>
#include "PCF8575.h"

PCF8575 pcf8575(0x21); //portexpander bus adresse und name 

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
HallPosition magnetPosition = LINKS;                                                        // MAGNET CONFIG HIER
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
int zielzoneMinWertRed = 70;     //zielzone geht von minwert bis maxwert
//int zielzoneMinWertGreen -->   Existiert nicht weil zu ungenau, könnte auch Boden sein!!
int zielzoneMinWertBlue = 60;

int zielzoneMaxWertRed = 110; 
//int zielzoneMaxWertGreen -->   Existiert nicht weil zu ungenau, könnte auch Boden sein!!
int zielzoneMaxWertBlue = 90;

int lineMinWertBlue = 135; 
int lineMinWertGreen = 180;
int lineMinWertRed = 180;

int lineMaxWertBlue = 300;
int lineMaxWertGreen = 300;
int lineMaxWertRed = 300;

int lineTempVarRed;
int lineTempVarGreen;
int lineTempVarBlue;

int istInModus=0;
//---------------------------------//

// - Funktionen -
//liniensensor
int readSensorLeft() { //sensor links
  return digitalRead(IR_LEFT);
}

int readSensorMiddle() { //sensor middle
  return digitalRead(IR_MIDDLE);
}

int readSensorRight() { //sensor rechts
  return digitalRead(IR_RIGHT);
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

int readMagnetSensor() {
  return digitalRead(HALL_SENSOR_D);
  if(readMagnetSensor() == 0) { //wenn magnet
    if(readMagnetSensor() == 0 && hallValAlt == 0) { //wenn davor auchs chon magnet war
      ledAn();
    }
  } else if(readMagnetSensor() == 1) {
    ledAus();
  }
}

int readLineColorSensor() {
  if(readBlueColor() > lineMinWertBlue && readBlueColor() < lineMaxWertBlue && readGreenColor() > lineMinWertGreen && readGreenColor() < lineMaxWertGreen && readRedColor() > lineMinWertRed && readRedColor() < lineMaxWertRed) {
    return 1;
  } else {
    return 0;
  }
}

int readRedColor() {
  pcf8575.digitalWrite(SENSOR_S2, LOW);
  pcf8575.digitalWrite(SENSOR_S3, LOW);
  return pulseIn(SENSOR_OUT, LOW);
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

//AUSGABENFUNKTINIEN

void ledAn() {
  pcf8575.digitalWrite(LED_PIN, LOW); //led wird angeschaltet
}

void ledAus() {
  pcf8575.digitalWrite(LED_PIN, HIGH); //led wird ausgeschaltet
}

void magnetLesen() {
  Hall_Val2=digitalRead(HALL_SENSOR_D);
  if(Hall_Val2 == 0) {
    Serial.print("Ja"); // das hier wird nur zum loggen ausgeführt, man kann danach die hall_val2 trz abfragen für eine andere aktion
    if(Hall_Val2 == 0 && hallValAlt == 0) {
      ledAn();
    }
  } else {
    Serial.print("Nein");
    ledAus();
  }
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
//old, dienen nur zur ausgabe
void entfernungMessenVorne() {
  pcf8575.digitalWrite(TRIGGER_VORNE, LOW); //Hier nimmt man die Spannung für kurze Zeit vom Trigger-Pin, damit man später beim Senden des Trigger-Signals ein rauschfreies Signal hat.
  delay(5); // Pause 5 Millisekunden
  pcf8575.digitalWrite(TRIGGER_VORNE, HIGH); //Jetzt sendet man eine Ultraschallwelle los.
  delay(10); //Dieser „Ton“ erklingt für 10 Millisekunden.
  pcf8575.digitalWrite(TRIGGER_VORNE, LOW);//Dann wird der „Ton“ abgeschaltet.
  dauerVorne = pulseIn(ECHO_VORNE, HIGH); //Mit dem Befehl „pulseIn“ zählt der Mikrokontroller die Zeit in Mikrosekunden, bis der Schall zum Ultraschallsensor zurückkehrt.
  entfernungVorne = (long)((dauerVorne/2) * 0.03432); //Nun berechnet man die Entfernung in Zentimetern. Man teilt zunächst die Zeit durch zwei (Weil man ja nur eine Strecke berechnen möchte und nicht die Strecke hin- und zurück). Den Wert multipliziert man mit der Schallgeschwindigkeit in der Einheit Zentimeter/Mikrosekunde und erhält dann den Wert in Zentimetern.
  
  if (entfernungVorne >= 500 || entfernungVorne <= 0) {//Wenn die gemessene Entfernung über 500cm oder unter 0cm liegt,…
    Serial.print("Vorne: ");
    Serial.print(entfernungVorne); //dann soll der serial monitor ausgeben „Kein Messwert“, weil Messwerte in diesen Bereichen falsch oder ungenau sind.
  }
  else {
    Serial.print("Vorne: ");
    Serial.print(entfernungVorne); //…soll der Wert der Entfernung an den serial monitor hier ausgegeben werden.
  //  Serial.println(" cm Vorne"); // Hinter dem Wert der Entfernung soll auch am Serial Monitor die Einheit "cm" angegeben werden, danach eine neue Zeile
  }
  Serial.print("cm   ");
}

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


void entfernungMessenRechts() {
  entfernungRechtsOld = entfernungRechts;
  pcf8575.digitalWrite(TRIGGER_RECHTS, LOW); //Hier nimmt man die Spannung für kurze Zeit vom Trigger-Pin, damit man später beim Senden des Trigger-Signals ein rauschfreies Signal hat.
  delay(5); // Pause 5 Millisekunden
  pcf8575.digitalWrite(TRIGGER_RECHTS, HIGH); //Jetzt sendet man eine Ultraschallwelle los.
  delay(10); //Dieser „Ton“ erklingt für 10 Millisekunden.
  pcf8575.digitalWrite(TRIGGER_RECHTS, LOW);//Dann wird der „Ton“ abgeschaltet.
  dauerRechts = pulseIn(ECHO_RECHTS, HIGH); //Mit dem Befehl „pulseIn“ zählt der Mikrokontroller die Zeit in Mikrosekunden, bis der Schall zum Ultraschallsensor zurückkehrt.
  entfernungRechts = (long)((dauerRechts/2) * 0.03432); //Nun berechnet man die Entfernung in Zentimetern. Man teilt zunächst die Zeit durch zwei (Weil man ja nur eine Strecke berechnen möchte und nicht die Strecke hin- und zurück). Den Wert multipliziert man mit der Schallgeschwindigkeit in der Einheit Zentimeter/Mikrosekunde und erhält dann den Wert in Zentimetern.
  
  if (entfernungRechts >= 500 || entfernungRechts <= 0) {//Wenn die gemessene Entfernung über 500cm oder unter 0cm liegt,…
    Serial.print("Rechts: ");
    Serial.print(entfernungRechts); //dann soll der serial monitor ausgeben „Kein Messwert“, weil Messwerte in diesen Bereichen falsch oder ungenau sind.
  }
  else {
    Serial.print("Rechts: ");
    Serial.print(entfernungRechts); //…soll der Wert der Entfernung an den serial monitor hier ausgegeben werden.
  //  Serial.println(" cm Rechts"); // Hinter dem Wert der Entfernung soll auch am Serial Monitor die Einheit "cm" angegeben werden, danach eine neue Zeile
  }
  Serial.print("cm   ");
}

void ausgabeRot() {
  redWert = readRedColor();
  Serial.print("   ");
  Serial.print("R= ");
  Serial.print(redWert);
}

void ausgabeGrun() {
  grunWert = readGreenColor();
  Serial.print("   ");
  Serial.print("G= ");
  Serial.print(grunWert);
}

void ausgabeBlue() {
  blueWert = readBlueColor();
  Serial.print("   ");
  Serial.print("B= ");
  Serial.print(blueWert);
}

void umdrehungZeitVoid() {
    delay(umdrehungZeit);                          //HIER ZEIT EINFÜGEN WIE LANG ES DAUERT FÜR EINE KURVE
}
// outdated
void kursUmdrehungZeit() { //Zeit um wieder auf den Kurs zu kommen
    delay(100);                          //HIER ZEIT EINFÜGEN WIE LANG ES DAUERT FÜR EINE UMDREHUNG
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
//Nicht benutzt
/*void kurzerAusgleichNachLinks() {
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
*/

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
  pcf8575.pinMode(TRIGGER_VORNE, OUTPUT); // Trigger-Pin ist ein Ausgang
  pinMode(ECHO_VORNE, INPUT); // Echo-Pin ist ein Eingang
  // Abstandssensor links
  pcf8575.pinMode(TRIGGER_LINKS, OUTPUT); // Trigger-Pin ist ein Ausgang
  pinMode(ECHO_LINKS, INPUT); // Echo-Pin ist ein Eingang
  // Abstandssensor rechts
  pcf8575.pinMode(TRIGGER_RECHTS, OUTPUT); // Trigger-Pin ist ein Ausgang
  pinMode(ECHO_RECHTS, INPUT); // Echo-Pin ist ein Eingang
  // Infrarotsensoren
  pinMode(IR_LEFT, INPUT); // sensor pin INPUT
  pinMode(IR_RIGHT, INPUT); // sensor pin INPUT
  pinMode(IR_MIDDLE, INPUT);
  // Hall Sensor
  pinMode(HALL_SENSOR_D,INPUT);
  //LED
  pcf8575.pinMode(LED_PIN, OUTPUT);

  //farbsensor
  pcf8575.pinMode(SENSOR_S0, OUTPUT); //portexpander
  pcf8575.pinMode(SENSOR_S1, OUTPUT); //portexpander
  pcf8575.pinMode(SENSOR_S2, OUTPUT); //portexpander
  pcf8575.pinMode(SENSOR_S3, OUTPUT); //portexpander
  pinMode(SENSOR_OUT, INPUT);

  // Setting frequency-scaling to 20% (farbsensor)
  pcf8575.digitalWrite(SENSOR_S0, HIGH); //portexpander
  pcf8575.digitalWrite(SENSOR_S1, LOW); //portexpander


  ////////////////////////////////////////////////////////////////////////////////////////////////
  pcf8575.begin(); //HIER DRUNTER KEIN PORTEXPANDER ZEUG MEHR, HIER WIRD BEGONNEN
  ////////////////////////////////////////////////////////////////////////////////////////////////

  Serial.println("----- INFO: Pins gesetzt");
  fahrenBeide(); //Bot startet das Fahren
  Serial.println("----- INFO: Im Setup Fahren gestartet");
//  delay(1500); //Delay dass nicht direkt irgendwelche Hindernisse erkannt werden
}


/*
 * Main Loop
 */
void loop() {
  durchgangCounter++; //eigentlich schwachsinn funktioniert aber
  //Serial.print(durchgangCounter);

  //ausgabe start
  entfernungMessenLinks(); // er misst durchgehend die entfernung nach vorne
  entfernungMessenVorne(); //entfernung links und rechts messen wenn vorne nh wand is
  entfernungMessenRechts();
  linieLinks();
  linieMitte();
  linieRechts();
  magnetLesen();
  hallValAlt = readMagnetSensor();
  hallValAlt2 = hallValAlt;
  ausgabeRot();
  ausgabeGrun();
  ausgabeBlue();

  //farbsensor zielzone
  if(readBlueColor() > lineMinWertBlue && readBlueColor() < lineMaxWertBlue && readGreenColor() > lineMinWertGreen && readGreenColor() < lineMaxWertGreen && readRedColor() > lineMinWertRed && readRedColor() < lineMaxWertRed) {
    Serial.println("  L"); //linienerkennung, variablen oben einstellbar
  }
  if(readRedColor() > zielzoneMinWertRed && readRedColor() < zielzoneMaxWertRed && readBlueColor() > zielzoneMinWertBlue && readBlueColor() < zielzoneMaxWertBlue) {
    Serial.println("  Z"); //linienerkennung, variablen oben einstellbar
    stehenbleiben();
    //exit(0);
  }

  //farbsensor linienfolgeskript
  while(readLineColorSensor() == 1) {
    istInModus++;
    if(istInModus == 1) {
      Serial.println("im Modus für Linie");
    }
    //speedsync soll laufen
    unsigned long currentMillis = millis(); //delay ohne delay
    if (currentMillis - previousMillis >= SPEEDSYNCINTERVAL  && readSensorMiddle() == 0) {
      previousMillis = currentMillis;
      if(readDistanceLeft() > readDistanceRight() || readDistanceRight() <= 8 || readDistanceRight() >= 45) { //größer als 45 weil so viel gar nicht sein kann, das ergebnis muss falsch sein
        outRight += 30;
        motorAnsteuernGeradeausLauf();
      }
      if(readDistanceRight() > readDistanceLeft() || readDistanceLeft() <= 8 || readDistanceLeft() >= 45) {
        outLeft += 30;
      motorAnsteuernGeradeausLauf();
      }
    }
    if(readSensorLeft() == 1 || readSensorRight() == 1) { //wenn irgendein infrarotsensor auf 1 ist und farbsensor auch
      if(readSensorLeft() == 1 || readLineColorSensor() == 1) {
        Serial.println("Halbumdrehung Links im Modus Linie");
        halbUmdrehungLinks();
      } else if(readSensorRight() == 1 || readLineColorSensor() == 1) {
        Serial.println("Halbumdrehung Rechts im Modus Linie");
        halbUmdrehungRechts();
      }
    }
    Serial.print(readSensorLeft());
    Serial.print("  ");
    Serial.print(readLineColorSensor());
    Serial.print("  ");
    Serial.print(readSensorRight());
    Serial.print("  ");
    Serial.print(readBlueColor());
    Serial.print("  ");
    Serial.print(readRedColor());
    Serial.print("  ");
    Serial.println(readGreenColor());
  }

  Serial.println("");
  //ausgabe ende


  //entfernung zu variable
  if (readDistanceFront() <= 23 && readDistanceFront() >= 1) { //wenn vorne eine wand ist dann fängt er an links und rechts zu messen
    stehenbleiben(); //direkt stehenbleiben
    if (readDistanceLeft() <= 23) { //wenn links eine wand ist wird hindernisLinks auf 1 gesetzt (wenn links weniger als 0 cm entfernt ist auch, also bei einem messfehler)
      hindernisLinks = 1;
    }
    if (readDistanceRight() <= 23) { //wenn rechts eine wand ist wird hindernisRechts auf 1 gesetzt
      hindernisRechts = 1;
    }

    //Entfernung Abfrage 
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

  //Linienabfrage

  if(readSensorLeft() == 1 && readSensorMiddle() == 1) {
      halbUmdrehungLinks();
  }
  if(readSensorRight() == 1 && readSensorMiddle() == 1) {
      halbUmdrehungRechts();
  }
  //nicht benötigt, funktioniert aber
  if(readSensorRight() == 0 && readSensorLeft() == 0 && readSensorMiddle() == 0) {
      fahrenBeide();
      motorAnsteuern();
  }


  //Speedsync
  unsigned long currentMillis = millis(); //delay ohne delay
  if (currentMillis - previousMillis >= SPEEDSYNCINTERVAL  && readSensorMiddle() == 0) {
    previousMillis = currentMillis;
    if(readDistanceLeft() > readDistanceRight() || readDistanceRight() <= 8 || readDistanceRight() >= 45) { //größer als 45 weil so viel gar nicht sein kann, das ergebnis muss falsch sein
      outRight += 30;
      motorAnsteuernGeradeausLauf();
    }
    if(readDistanceRight() > readDistanceLeft() || readDistanceLeft() <= 8 || readDistanceLeft() >= 45) {
      outLeft += 30;
      motorAnsteuernGeradeausLauf();
    }
  }

  //Magnetskript
  if(readMagnetSensor() == 0) { //wenn ein magnet erkannt wird
    if(hallValAlt == 0 && readMagnetSensor() == 0 && hallValAlt2 == 0) { //wenn wirklich ein magnet da ist, 3 mal hintereinander ist
      ledAn();
      if(magnetPosition == RECHTS) {
        halbUmdrehungLinks();
      } else if(magnetPosition == LINKS) {
        halbUmdrehungRechts();
      }
    }
  } else{ //wenn kein magnet da is
    ledAus();
  }
  

  hindernisLinks = 0;
  hindernisRechts = 0;
  hallValAlt2 = 1;  //müssen wir leider auf 1 setzten weil magnet = 0
  hallValAlt = 1;   //müssen wir leider auf 1 setzten weil magnet = 0
  istInModus = 0;   //dass wir kein spam haben und die ausgabe im modus wieder geht amogus ist btw sus
} 