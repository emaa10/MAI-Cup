#include <TaskMacro.h>
void setup() {
  Serial.begin(9600);
}

void loop() {
  taskBegin();
  Serial.println("task");
  delay(1000);
  taskEnd();
  Serial.print("loop");
}