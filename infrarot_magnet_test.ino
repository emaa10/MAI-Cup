int ir_left = A3; // connect ir sensor to arduino pin 2 (left one)

void setup() 
{
  pinMode (ir_left, INPUT); // sensor pin INPUT
  Serial.begin(9600);
}

void loop()
{
  int statusSensorLeft = digitalRead (ir_left);
  
  if (statusSensorLeft == 1) {
    Serial.println("Left kein Hindernis");
  }
  
  else
  {
    Serial.println("Left Hindernis issda");
  }
  delay(200);
}