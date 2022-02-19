int ir_left = A3; // connect ir sensor to arduino pin 2 (left one)
int ir_right = A1;

#define Hall_Sensor A0          //A0 used with analog output, D2 with digital output
#define Hall_Sensor_D A2
int Hall_Val1=0,Hall_Val2=0;

void setup() 
{
  pinMode(ir_left, INPUT); // sensor pin INPUT
  pinMode(ir_right, INPUT); // sensor pin INPUT
  pinMode(Hall_Sensor_D,INPUT);
  Serial.begin(9600);
}

void magnetLesen() {
  Hall_Val2=digitalRead(Hall_Sensor_D);
  if(Hall_Val2 == 0) {
    Serial.print("MAGNET GESICHTET");
  }
  Serial.println("\t");
}

void linieLinks() {
  int statusSensorLeft = digitalRead(ir_left);
  if(statusSensorLeft == 1) {
    Serial.print("Left kein Hindernis");
    Serial.print("\t");
  } else {
    Serial.print("Left Hindernis issda");
    Serial.print("\t");
  }
}

void linieRechts() {
  int statusSensorRight = digitalRead(ir_right);
  if(statusSensorRight == 1) {
    Serial.print("Rechts kein Hindernis");
    Serial.print("\t");
  } else
    Serial.print("Rechts Hindernis issda");
    Serial.print("\t");
}

void loop()
{
  linieLinks();
  linieRechts();
  magnetLesen();
  delay(200);
} 