#include "core_pins.h"
#include <Arduino.h>

void setup() {
  Serial.begin(38400);
  pinMode(A0, INPUT_PULLDOWN);
  analogReadResolution(12);
  analogReadAveraging(16);
  analogReference(0);

}

void loop() {
  int val = analogRead(A0);
  float val_volt = val * 3.3 / 1024;
  Serial.print("analog value in volt: ");
  Serial.println(val_volt);
  delay(250);
}
