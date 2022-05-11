#include <Arduino.h>

int outleft = 0;
int outright = 0;
int Farbe = 0;
int skip = 1;

void motorAnsteuern() {
  unsigned long currentMillis = millis(); //delay ohne delay
  /*
  Alle 100 ms (MOTORSPEEDSYNCINTERVAL) wird der Sollwert mit dem Istwert verglichen.
  Wenn der Sollwert größer ist, wird der Istwert um einen kleinen Schritt (5, MOTORSPEEDSYNCSTEP) erhöht.
  Wenn er kleiner ist, wird er sofort angepasst. 
  Warum das Ganze? Weil wir sonst große Sprünge im Strom haben, was unsere kleine Batterie überfordert. Die Folge: Reboots
  */
  if (currentMillis - previousMillisMotor >= MOTORSPEEDSYNCINTERVAL) {
    previousMillisMotor = currentMillis;
    if (outRight != motorRight) {
      if (outRight > motorRight) {
        motorRight += MOTORSPEEDSYNCSTEP;
      } else {
        motorRight = outRight;
      }
    }
    if (outLeft != motorLeft) {
      if (outLeft > motorLeft) {
        motorLeft += MOTORSPEEDSYNCSTEP;
      } else {
        motorLeft = outLeft;
      }
    }
  }
  analogWrite(RIGHT_LPWM,motorRight); //Schreibe Geschwindigkeit auf Pins
  analogWrite(RIGHT_RPWM,0);          //Schreibe Geschwindigkeit auf Pins
  analogWrite(LEFT_LPWM,motorLeft);   //Schreibe Geschwindigkeit auf Pins
  analogWrite(LEFT_RPWM,0);           //Schreibe Geschwindigkeit auf Pins
}

void motorAnsteuernGeradeausLauf() {
  if(outLeft >= 70 || outRight >= 70) {
    outLeft -= 30;
    outRight -= 30;
  }
  motorAnsteuern();
}

//AUSGABENFUNKTINIEN

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
  outLeft = 40;
  outRight = 40;
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
  digitalWrite(TRIGGER_VORNE, LOW); //Hier nimmt man die Spannung für kurze Zeit vom Trigger-Pin, damit man später beim Senden des Trigger-Signals ein rauschfreies Signal hat.
  delay(5); // Pause 5 Millisekunden
  digitalWrite(TRIGGER_VORNE, HIGH); //Jetzt sendet man eine Ultraschallwelle los.
  delay(10); //Dieser „Ton“ erklingt für 10 Millisekunden.
  digitalWrite(TRIGGER_VORNE, LOW);//Dann wird der „Ton“ abgeschaltet.
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
  digitalWrite(TRIGGER_LINKS, LOW); //Hier nimmt man die Spannung für kurze Zeit vom Trigger-Pin, damit man später beim Senden des Trigger-Signals ein rauschfreies Signal hat.
  delay(5); // Pause 5 Millisekunden
  digitalWrite(TRIGGER_LINKS, HIGH); //Jetzt sendet man eine Ultraschallwelle los.
  delay(10); //Dieser „Ton“ erklingt für 10 Millisekunden.
  digitalWrite(TRIGGER_LINKS, LOW);//Dann wird der „Ton“ abgeschaltet.
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
  digitalWrite(TRIGGER_RECHTS, LOW); //Hier nimmt man die Spannung für kurze Zeit vom Trigger-Pin, damit man später beim Senden des Trigger-Signals ein rauschfreies Signal hat.
  delay(5); // Pause 5 Millisekunden
  digitalWrite(TRIGGER_RECHTS, HIGH); //Jetzt sendet man eine Ultraschallwelle los.
  delay(10); //Dieser „Ton“ erklingt für 10 Millisekunden.
  digitalWrite(TRIGGER_RECHTS, LOW);//Dann wird der „Ton“ abgeschaltet.
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

void liniefolgenmitFarbsensor() {
    skip = 0;
    for(int i=0;i<20;i+5 || skip == 1){
        outright  = 0;
        outleft = 40;
        if (Farbe ==  3); { //3 ist in diesem Fall schwarz
            skip = 1;
            break;
        }
    }
}
    






void umdrehungZeitVoid() {
    delay(umdrehungZeit);                          //HIER ZEIT EINFÜGEN WIE LANG ES DAUERT FÜR EINE KURVE
}
// outdated
void kursUmdrehungZeit() { //Zeit um wieder auf den Kurs zu kommen
    delay(100);                          //HIER ZEIT EINFÜGEN WIE LANG ES DAUERT FÜR EINE UMDREHUNG
}

void halbUmdrehungRechts() { //Quasi 90* Drehung nach rechts
  stehenbleiben();
  outLeft = umdrehungSpeed;
  outRight = 0;
  motorAnsteuern();
  delay(umdrehungZeit);
  outLeft = 0;
  motorAnsteuern();
}

void halbUmdrehungLinks() { //Quasi 90* Drehung nach links
  stehenbleiben();
  outLeft = 0;
  outRight = umdrehungSpeed;
  motorAnsteuern();
  delay(umdrehungZeit);
  outRight = 0;
  motorAnsteuern();
}


// ------------------------------------------------------------------------------------
// -                                Ende der Methoden                                 -
// ------------------------------------------------------------------------------------


