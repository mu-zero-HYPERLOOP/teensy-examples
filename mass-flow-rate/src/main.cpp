#include <Arduino.h>

const byte massFlowPin = 2;
const int flow = 100; //100ml

volatile unsigned int presentRPM = 0;

void massFlowISR() {
  static unsigned long previousMicros = micros();                 // remember variable, initialize first time
  unsigned long presentMicros = micros();                         // read microseconds
  unsigned long revolutionTime = presentMicros - previousMicros;  // works fine with wrap-around of micros()
  if (revolutionTime < 1000UL) return;                            // avoid divide by 0, also debounce, speed can't be over 60,000
  presentRPM = (60000000UL / revolutionTime);                     // calculate
  previousMicros = presentMicros;                                 // store microseconds for next time
}

int getFlowRate() {
    return flow / presentRPM;
}

void setup() {
    pinMode(massFlowPin, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(massFlowPin), massFlowISR, CHANGE);
}


void loop() {
    delay(100);
}