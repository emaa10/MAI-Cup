#include <Arduino.h>

#define RIGHT_RPWM 5
#define RIGHT_LPWM 6
#define RIGHT_REN 8
#define RIGHT_LEN 9

#define LEFT_RPWM 10
#define LEFT_LPWM 11
#define LEFT_REN 2
#define LEFT_LEN 3
#define LED_PIN 4

#define TRIGGER 13
#define ECHO 12

int outLeft;
int outRight;
long dauer=0;
long entfernung=0;
int Zeitfahren=0;
int umdrehungZeit=840;
int umdrehungSpeed=110;
int Logic=0; 

void ledAn() {
  digitalWrite(LED_PIN, LOW); 
}
void ledAus() {
  digitalWrite(LED_PIN, HIGH); 
}

void motorAnsteuern() {
  analogWrite(RIGHT_LPWM,outRight);
  analogWrite(RIGHT_RPWM,0);
  analogWrite(LEFT_LPWM,outLeft);
  analogWrite(LEFT_RPWM,0);
}

void fahrenBeide() {
    outLeft = 100;
    outRight = 100;
    motorAnsteuern();
}

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

void entfernungmessen(){
    digitalWrite(TRIGGER, LOW);              
                                             
                                             
    delay(5);                                
    digitalWrite(TRIGGER, HIGH);             
                                             
    delay(10);                               
    digitalWrite(TRIGGER, LOW);              
                                             
    dauer = pulseIn(ECHO, HIGH);             
                                             
    entfernung = (dauer/2) / 29.1;           

        Serial.print(entfernung);            
        Serial.println(" cm");               

    delay(30);                             
}

void setup(){
    Serial.begin(9600);                 
    pinMode(TRIGGER, OUTPUT);           
    pinMode(ECHO, INPUT);               
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

    outLeft = 0;
    outRight = 100;
}
void loop(){
    entfernungmessen();
    if (entfernung <= 20) {
        for (int y = 0; y <= 15; y++) {
            halbUmdrehungLinks(),
        }
    }
}