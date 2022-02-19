#define IR_LEFT = A3 // connect ir sensor to arduino pin 2 (left one)
#define IR_MIDDLE = A2 // connect ir sensor to arduino pin 2 (middle one)
#define IR_RIGHT = A1 // connect ir sensor to arduino pin 2 (middle one)
int statusSensorLeft;
int statusSensorMiddle;
int statusSensorRight;

void setup() 
{
  pinMode (IR_LEFT, INPUT); // sensor pin INPUT
  pinMode (IR_MIDDLE, INPUT); // sensor pin INPUT
  pinMode (IR_RIGHT, INPUT); // sensor pin INPUT
  Serial.begin(9600);
}

void linieLinks() {
    statusSensorLeft = digitalRead (IR_LEFT);
    if (statusSensorLeft == 1) {
      Serial.println("Left kein Hindernis");
    }
    else {
      Serial.println("Left Hindernis issda");
    }
}

void linieMitte() {
    statusSensorMiddle = digitalRead (IR_MIDDLE);
    if (statusSensorMiddle == 1) {
      Serial.println("Mitte kein Hindernis");
    }
    else {
      Serial.println("Mitte Hindernis issda");
    }
}

void linieRechts() {
    statusSensorRight = digitalRead (IR_RIGHT);
    if (statusSensorRight == 1) {
      Serial.println("Rechts kein Hindernis");
    }
    else {
      Serial.println("Rechts Hindernis issda");
    }
}


void loop()
{
  linieLinks();
  linieMitte();
  linieRechts();
  delay(200);
}