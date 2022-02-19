int ir_left = A3; // connect ir sensor to arduino pin 2 (left one)
int ir_middle = A2;
int ir_right = A1;


void setup() 
{
  pinMode (ir_left, INPUT); // sensor pin INPUT
  pinMode (ir_middle, INPUT); // sensor pin INPUT
  pinMode (ir_right, INPUT); // sensor pin INPUT
  Serial.begin(9600);
}

void loop()
{
  int statusSensorLeft = digitalRead (ir_left);
  int statusSensorMiddle = digitalRead (ir_middle);
  int statusSensorRight = digitalRead (ir_right);

  if (statusSensorLeft == 1) {
    Serial.println("Left kein Hindernis");
  }

  else
  {
    Serial.println("Left Hindernis issda");
  }
  if (statusSensorMiddle == 1) {
    Serial.println("MItte kein Hindernis");
  }

  else
  {
    Serial.println("Mitte Hindernis issda");
  }
  
  if (statusSensorRight == 1) {
    Serial.println("Recgts kein Hindernis");
  }

  else
  {
    Serial.println("Rechts Hindernis issda");
  }
  
  delay(200);
} 