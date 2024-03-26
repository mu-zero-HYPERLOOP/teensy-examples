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
    delay(100);
}

void detectRisingEdge(){
    ++count;
    delay(100);
}

void detectFallingEdge(){
    ++count;
    Serial.println(count);
    if (count % 2 == 1) {
        Serial.println(errorMsg);
    }
    delay(100);
}

void loop() {

}
