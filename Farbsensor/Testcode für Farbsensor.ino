#include <Arduino.h>
#define SENSOR_S0 3
#define SENSOR_S1 2
#define SENSOR_S2 4
#define SENSOR_S3 5
#define SENSOR_OUT 6

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