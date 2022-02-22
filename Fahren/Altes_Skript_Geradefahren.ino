
  if (entfernungRechtsOld > entfernungRechts) { //er speichert alle 200ms die alte und die neue entfernung und vergleicht beide variablen dann. wenn rechts vorher weiter weg war, nähert er sich nach rechts an und fährt nun nach links (nur kurz). umgekehrt halt genauso
    kurzerAusgleichNachLinks();
    //Serial.println("----- INFO: Rechts war davor weiter weg, daher fährt er kurz nach links");
  }
  if (entfernungRechts > entfernungRechtsOld) {
    kurzerAusgleichNachRechts();
    //Serial.println("----- INFO: Links war davor weiter weg, daher fährt er kurz nach rechts");
  }
  int statusSensorLeft = digitalRead(ir_left);
  int statusSensorRight = digitalRead(ir_right);
  //linienabfrage:
  if (statusSensorLeft == 1) { //wenn sensor links auf linie ist
    halbUmdrehungLinks(); //soll er nach links fahren weil er ja nur die abbiegung wahrnimmt
    Serial.println("Fährt 90° nach Links weil Linie");
  }
  if (statusSensorRight == 1) {
    halbUmdrehungRechts();
    Serial.println("Fährt 90° nach Rechts weil Linie");
  }
