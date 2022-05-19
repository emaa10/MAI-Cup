#include <Arduino.h>
#define SENSOR_S0 3
#define SENSOR_S1 2
#define SENSOR_S2 4
#define SENSOR_S3 5
#define SENSOR_OUT 6

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

int outleft = 0;
int outright = 0;
int Farbe = 0; //Farben Index: 1 = Rot, 2 = Grün, 3 = Schwarz, 4 = Blau
int skip = 0;
int skip2 = 0;
int skip3 = 0;
int frequency = 0; // Für Farbsensor

  void Farbemessen() {
  // Setting RED filtered photodiodes to be read
  digitalWrite(SENSOR_S2, LOW);
  digitalWrite(SENSOR_S3, LOW);
  frequency = pulseIn(SENSOR_OUT, LOW); // Reading the output frequency
  // Farbe auslesung RED
  if (frequency < 300) {
    Farbe = 1;
  }
  delay(100);

  // Setting GREEN filtered photodiodes to be read
  digitalWrite(SENSOR_S2, HIGH);
  digitalWrite(SENSOR_S3, HIGH);
  frequency = pulseIn(SENSOR_OUT, LOW);
  // Farbe auslesung GREEN
  if (frequency < 300) {
    Farbe = 2;
  }
  delay(100);

  // Setting BLUE filtered photodiodes to be read
  digitalWrite(SENSOR_S2, LOW);
  digitalWrite(SENSOR_S3, HIGH);
  frequency = pulseIn(SENSOR_OUT, LOW);
  // Farbe auslesung BLUE
  if (frequency < 300) {
    Farbe = 4;
  }
  delay(100);
  }
}

void Farbeausgeben() {
  if (Farbe == 1) {
    Serial.println("Rot");
  }
  if (Farbe == 2) {
    Serial.println("Grün");
  }
  if (Farbe == 3) {
    Serial.println("Schwarz");
  }
  if (Frabe == 4) {
    Serial.println("Blau");
  }
}

void Frabemessenbesser() {
  outleft = 0;
  outright = 0;
  motorAnsteuern();
  for(int i=0;i<4;i+1 ) {
    Farbemessen();
    delay(10);
  }
}
void setup()
{
  pinMode(SENSOR_S0, OUTPUT);
  pinMode(SENSOR_S1, OUTPUT);
  pinMode(SENSOR_S2, OUTPUT);
  pinMode(SENSOR_S3, OUTPUT);
  pinMode(SENSOR_OUT, INPUT);

  // Setting frequency-scaling to 20%
  digitalWrite(SENSOR_S0, HIGH);
  digitalWrite(SENSOR_S1, LOW);

  Serial.begin(9600);
}

void loop() {
  Farbemessen();
  Farbeausgeben();
  delay(100);
}