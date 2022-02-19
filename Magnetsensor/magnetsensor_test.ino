//test v1 - noch nichts besonderes

//int statusLed = 12 ; // PIN für die LED zur anzeige des Sensor zustandes
int magnetSensor = A0; // PIN für den Magnetischen Hall Sensors
 
void setup () {
  pinMode (magnetSensor, INPUT);  // definieren des PIN's für den Sensor als Eingangssignal
  Serial.begin(9600);
}
 
void loop (){
int magnetSignal = digitalRead (magnetSensor) ; // Lesen des Zustandes des Sensors.
  if (magnetSignal == LOW){ //Wenn dieser AN ist dann soll die StatusLed leuchten.
    Serial.println("led high");
//    digitalWrite (statusLed, HIGH);
  } 
  else { //Wenn dieser AUS ist dann soll die StatusLed NICHT leuchten.
//    digitalWrite (statusLed, LOW);
    Serial.println("led low");
  }
}