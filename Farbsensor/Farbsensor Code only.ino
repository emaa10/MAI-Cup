#include <Arduino.h>
#define SENSOR_S0 3
#define SENSOR_S1 2
#define SENSOR_S2 4
#define SENSOR_S3 5
#define SENSOR_OUT 6

int frequency = 0;

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

void loop()
{
  // Setting RED filtered photodiodes to be read
  digitalWrite(SENSOR_S2, LOW);
  digitalWrite(SENSOR_S3, LOW);
  frequency = pulseIn(SENSOR_OUT, LOW); // Reading the output frequency

  // Printing RED value on the serial monitor
  Serial.print("R=");
  Serial.print(frequency);
  Serial.print("  ");
  delay(100);

  // Setting GREEN filtered photodiodes to be read
  digitalWrite(SENSOR_S2, HIGH);
  digitalWrite(SENSOR_S3, HIGH);
  frequency = pulseIn(SENSOR_OUT, LOW);

  // Printing GREEN value on the serial monitor
  Serial.print("G=");
  Serial.print(frequency);
  Serial.print("  ");
  delay(100);

  // Setting BLUE filtered photodiodes to be read
  digitalWrite(SENSOR_S2, LOW);
  digitalWrite(SENSOR_S3, HIGH);
  frequency = pulseIn(SENSOR_OUT, LOW);

  // Printing BLUE value on the serial monitor
  Serial.print("B=");
  Serial.print(frequency);
  Serial.println("  ");
  delay(100);
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