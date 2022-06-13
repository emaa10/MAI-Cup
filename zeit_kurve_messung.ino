#include <Arduino.h>

   /*
  BTS7960-43A-Driver
  made on 22 Nov 2020
  by Amir Mohammad Shojaee @ Electropeak
  Home

*/

#define RIGHT_RPWM 5
#define RIGHT_LPWM 6
#define RIGHT_REN 8
#define RIGHT_LEN 9

#define LEFT_RPWM 10
#define LEFT_LPWM 11
#define LEFT_REN 2
#define LEFT_LEN 3

int outLeft;
int outRight;

int umdrehungSpeed=110;
int umdrehungZeit=850;

void setup() {
  Serial.begin(9600);
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
  delay(2000);

  analogWrite(RIGHT_LPWM,umdrehungSpeed);
  analogWrite(RIGHT_RPWM,0);
  analogWrite(LEFT_LPWM,umdrehungSpeed);
  analogWrite(LEFT_RPWM,0);
  delay(umdrehungZeit);                   //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

  analogWrite(RIGHT_LPWM,0);
  analogWrite(RIGHT_RPWM,0);
  analogWrite(LEFT_LPWM,0);
  analogWrite(LEFT_RPWM,0);
  delay(1000);
  

}
 
 
void loop() {
  //nix mehr
  analogWrite(RIGHT_LPWM,0);
  analogWrite(RIGHT_RPWM,0);
  analogWrite(LEFT_LPWM,0);
  analogWrite(LEFT_RPWM,0);

}
