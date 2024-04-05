#include <Arduino.h>

const byte massFlowPin = 2;

void massFlowISR() {

}

void setup() {
    pinMode(massFlowPin, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(massFlowPin), massFlowISR, CHANGE);
}


void loop() {
    delay(100);
}