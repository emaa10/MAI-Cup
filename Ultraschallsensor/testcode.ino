#define TRIGGER 4
#define ECHO 3
long dauer=0;
long entfernung=0;
void setup() {
    Serial.begin(9600);                 // Die serielle Kommunikation starten
    pinMode(TRIGGER, OUTPUT);           // Trigger Pin als Ausgang definieren
    pinMode(ECHO, INPUT);               // Echo Pin als Eingang defnieren
}

void loop() {
    digitalWrite(trigger, LOW);              // Den Trigger auf LOW setzen um
                                             // ein rauschfreies Signal
                                             // senden zu können
    delay(5);                                // 5 Millisekunden warten
    digitalWrite(trigger, HIGH);             // Den Trigger auf HIGH setzen um eine 
                                             // Ultraschallwelle zu senden
    delay(10);                               // 10 Millisekunden warten
    digitalWrite(trigger, LOW);              // Trigger auf LOW setzen um das 
                                             // Senden abzuschließen
    dauer = pulseIn(echo, HIGH);             // Die Zeit messen bis die 
                                             // Ultraschallwelle zurückkommt
    entfernung = (dauer/2) / 29.1;           // Die Zeit in den Weg in Zentimeter umrechnen

        Serial.print(entfernung);            // Den Weg in Zentimeter ausgeben
        Serial.println(" cm");               //

    delay(1000);                             // Nach einer Sekunde wiederholen

}