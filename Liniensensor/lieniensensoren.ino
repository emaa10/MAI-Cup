#include <Arduino.h>
/*@Mai Cup Junior
made by Christoph
at 24.02.2022
line sensonic
*/
define RIGHT_RPWM 5//ales halt von definieren und so damit kein fehler
#define RIGHT_LPWM 6
#define RIGHT_REN 8
#define RIGHT_LEN 9
//Motor links
#define LEFT_RPWM 10
#define LEFT_LPWM 11
#define LEFT_REN 2
#define LEFT_LEN 3
// Ultraschall vorne/MITTE
#define TRIGGER_VORNE 12 //Ultraschallsensor vorne Trigger Pin 
#define ECHO_VORNE 13    //Ultraschallsensor vorne Echo Pin 
// Ultraschall links
#define TRIGGER_LINKS 7 
#define ECHO_LINKS A5
//Ultraschall rechts
#define TRIGGER_RECHTS 4 
#define ECHO_RECHTS A4
//Hall Sensor
#define HALL_SENSOR A0          //analog output (optional)
#define HALL_SENSOR_D A2        // digital output (benutzt zum auslesen ob magnet oder nd)
//Infrarot Sensor
int ir_left = A3; // connect ir sensor to arduino pin 2 (left one)
int ir_right = A1;


// - Daten -
//motor
int outLeft; 
int outRight;
// alte trash logic dinger
int logicRight; //Temporärer Speicher bei der Kurve
int logicLeft; //Temporärer Speicher bei der Kurve
int logicRight1; //Temporärer Speicher bei der Kurve
int logicLeft1; //Temporärer Speicher bei der Kurve
int logicRight2; //Temporärer Speicher bei der Kurve
int logicLeft2; //Temporärer Speicher bei der Kurve
// neue logic checks
int hindernisLinks;
int hindernisRechts;
//Hall Sensor
int Hall_Val1=0,Hall_Val2=0;
// Ultraschall
long dauerVorne=0; // Dauer Speicher für Ultraschcallsensor vorne
long entfernungVorne=0; // Entfernung Speicher für Ultraschcallsensor vorne

long dauerLinks=0; 
long entfernungLinks=0; 

long dauerRechts=0; 
long entfernungRechts=0;

int entfernungLinksOld; //alte variable wird hier gespeichert
int entfernungRechtsOld;
//technik
int durchgangCounter=0;

/*------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
                                                                    Anfang der Methoden
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void umdrehungZeit() {
    delay(830);                          //HIER ZEIT EINFÜGEN WIE LANG ES DAUERT FÜR EINE KURVE
}
void linieLinks() { // Sensor links
  int statusSensorLeft = digitalRead(ir_left);
  if(statusSensorLeft == 1) { // diese abfrage kann man später auch noch verwenden
    Serial.print("Linie");
    Serial.print("   "); // serial monitor bestätigt kann mit fahr code anfangen
    outLeft = 0;//setz die Motoren auf die drehung
    outRight = 200;
    for (830){ //startet die Schleife um sich um 90* zu drehen, macht dies 830 mal da in der schleife ein delay von 1 drin ist
        delay(1);// das besagte delay
        if (statusSensorRight == 1) {//check in der Schleife ob nicht der andere sensor die Linie erkennnt, denn das würde bedeuten das er dieser noch folgen müsste
            for (statusSensorLeft == 1) {//wenn eben dieser Fall eintritt dreht er sich wieder zurück auf die linie und da es im loop ist wird dadurch die linie auch verfolgt
                outLeft = 200;//gibt den Motor das Signal sich wieder zurück zu drehen und weiter der Linie zu folgen
                outRight = 0;}
            outLeft = 0;//er richtet sich wieder aus um die Linie gerade zu verfolgen
            outRight = 200;
            delay(30);//Zeit der drehung zum wieder auf linie ausrichten idk
    {
        /* code */
    }
    
  } else {
    Serial.print("Boden");
    Serial.print("   ");
  }

void linieRechts() { // Sensor rechts
  int statusSensorRight = digitalRead(ir_right);
  if(statusSensorRight == 1) {
    Serial.print("Linie");
    Serial.print("   ");
    outLeft = 200;//setz die Motoren auf die drehung
    outRight = 0;
    for (830){//startet die Schleife um sich um 90* zu drehen, macht dies 830 mal da in der schleife ein delay von 1 drin ist
        delay(1);// das besagte delay
        if (statusSensorLeft == 1) {//check in der Schleife ob nicht der andere sensor die Linie erkennnt, denn das würde bedeuten das er dieser noch folgen müsste
            for (statusSensorRight == 1) {//wenn eben dieser Fall eintritt dreht er sich wieder zurück auf die linie und da es im loop ist wird dadurch die linie auch verfolgt
                outLeft = 0;//gibt den Motor das Signal sich wieder zurück zu drehen und weiter der Linie zu folgen
                outRight = 200;}
            outLeft =200;//er richtet sich wieder aus um die Linie gerade zu verfolgen
            outRight = 0;
            delay(30);//Zeit der drehung zum wieder auf linie ausrichten idk

  } else
    Serial.print("Boden");
    Serial.print("   ");
}