#include <Arduino.h>

const byte interruptPin = 2;
const String errorMsg = "Missing edge detected!";
int count = 0;

void setup() {
    pinMode(interruptPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(interruptPin), detectRisingEdge, RISING);
    attachInterrupt(digitalPinToInterrupt(interruptPin), detectFallingEdge, FALLING);
}

void increaseCount() {
    ++count;
    Serial.println(count);
}

void detectRisingEdge(){
    ++count;
}

void detectFallingEdge(){
    ++count;
    Serial.println(count);
    if (count % 2 == 1) {
        Serial.println(errorMsg);
    }
}

void loop() {

}
