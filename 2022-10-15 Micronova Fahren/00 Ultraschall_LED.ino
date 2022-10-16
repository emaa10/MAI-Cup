#define TRIGGER 13
#define ECHO 12
#define LED_PIN 4

void ledAn() {
  digitalWrite(LED_PIN, HIGH); 
}
void ledAus() {
  digitalWrite(LED_PIN, LOW); 
}


long dauer=0;
long entfernung=0;
void setup() {
    Serial.begin(9600);                 // Die serielle Kommunikation starten
    pinMode(TRIGGER, OUTPUT);           // Trigger Pin als Ausgang definieren
    pinMode(ECHO, INPUT);               // Echo Pin als Eingang defnieren
}

void loop() {
    digitalWrite(TRIGGER, LOW);              // Den Trigger auf LOW setzen um
                                             // ein rauschfreies Signal
                                             // senden zu können
    delay(5);                                // 5 Millisekunden warten
    digitalWrite(TRIGGER, HIGH);             // Den Trigger auf HIGH setzen um eine 
                                             // Ultraschallwelle zu senden
    delay(10);                               // 10 Millisekunden warten
    digitalWrite(TRIGGER, LOW);              // Trigger auf LOW setzen um das 
                                             // Senden abzuschließen
    dauer = pulseIn(ECHO, HIGH);             // Die Zeit messen bis die 
                                             // Ultraschallwelle zurückkommt
    entfernung = (dauer/2) / 29.1;           // Die Zeit in den Weg in Zentimeter umrechnen

        Serial.print(entfernung);            // Den Weg in Zentimeter ausgeben
        Serial.println(" cm");  
        Serial.println("");//

    delay(150);                             // Nach einer Sekunde wiederholen
    if(entfernung <= 30) {
        ledAn();
    } else{
        ledAus();
    }
}