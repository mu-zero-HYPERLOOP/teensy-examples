#include <Arduino.h>

void setup() {
  Serial.begin(38400);
}

void loop() {
  int val = analogRead(A0);
  float val_volt = val * 3.3 / 1024;
  Serial.print("analog value in volt: ");
  Serial.print(val_volt);
  delay(250);
}
