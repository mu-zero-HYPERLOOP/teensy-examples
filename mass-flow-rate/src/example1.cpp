#include <Arduino.h>

constexpr const int rxpin = 16;

volatile long timer = 0;
long average = 0.0;

void handleInterrupt() {
    timer = micros();
}

void setup() {
  pinMode(rxpin, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(rxpin), handleInterrupt, CHANGE);
}

// calculate period width in microseconds
void loop() {
  static long lastTimer = 0, counter = 0;
  static long printer = millis();
  // if had inperrupt on pin (timer>0.0) get pulse width time
  if (timer > 0.0) {
    // add pulse width time in microseconds to average and increment counter
    if (lastTimer) average += (timer - lastTimer);
    counter++;
    lastTimer = timer;  // note current time
    timer = 0;          // clear ready for nect reading
  }
  // after 10 seconds print average pulse width time
  if (millis() - printer > 10000) {
    float period = (float)average / counter;
    Serial.printf("Period %.2fuSec Frequency %.2fHz\n", period, 1000000.0f / period);  // print average pulse width
    counter = average = 0;                                                 // reset initialse values
    printer = millis();
  }
}