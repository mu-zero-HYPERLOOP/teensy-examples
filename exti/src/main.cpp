#include "core_pins.h"
#include <Arduino.h>

const byte interruptPin2 = 2;
const byte interruptPin3 = 3;
const String errorMsg = "Missing edge detected!";
int count = 0;

void increaseCount() {
    ++count;
    delay(100);
}

void detectPin2(){
  Serial.print("from rising ");
  Serial.println(count);
    ++count;
    delay(100);
}

void detectPin3(){
  Serial.print("from falling ");
  Serial.println(count);
    ++count;
    delay(100);
}
void detectFallingEdge2(){
  Serial.print("from falling ");
  Serial.println(count);
    ++count;
    delay(100);
}

void setup() {
    pinMode(interruptPin2, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(interruptPin2), detectPin2, CHANGE);
    pinMode(interruptPin3, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(interruptPin3), detectPin3, CHANGE);
}



void loop() {
  delay(100);

}
