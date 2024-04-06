#include <Arduino.h>

constexpr const int high = 1;
constexpr const int low = 0;

const byte edgeDetectionPin = 2;
const byte directionDetectionPin = 3;
const byte driveLeftPin = 4;
const byte driveRightPin = 5;

volatile byte edgeDetectorIndex = 1;
volatile bool directionDetectorIndex = 0;

String direction = "steady";
const int stripes = 5;
int track[2*stripes + 1]; // [LOW, HIGH, LOW, HIGH, LOW, HIGH, LOW, HIGH, LOW, HIGH, LOW]
int trackLength = 2*stripes + 1;

// Addressing edgeDetectionPin and directionDetectionPin

void readDirectionOnRising() {
    int val = digitalReadFast(directionDetectionPin);
    direction = (val == high) ? "left" : "right";
    attachInterrupt(digitalPinToInterrupt(edgeDetectionPin), readDirectionOnFalling, FALLING);
}

void readDirectionOnFalling() {
    int val = digitalReadFast(directionDetectionPin);
    direction = (val == high) ? "right" : "left";
    attachInterrupt(digitalPinToInterrupt(edgeDetectionPin), readDirectionOnRising, RISING);
}


// Simulate a track

void driveRight() {
    edgeDetectorIndex = std::min(edgeDetectorIndex + 1, trackLength - 1);
    directionDetectorIndex = std::min(directionDetectorIndex + 1, trackLength - 2);
    digitalWrite(directionDetectionPin, track[directionDetectorIndex]);
    digitalWrite(edgeDetectionPin, track[edgeDetectorIndex]);
}

void driveLeft() {
    edgeDetectorIndex = std::max(edgeDetectorIndex - 1, 1);
    directionDetectorIndex = std::max(directionDetectorIndex - 1, 0);
    digitalWrite(directionDetectionPin, track[directionDetectorIndex]);
    digitalWrite(edgeDetectionPin, track[edgeDetectorIndex]);
}

void createTrack() {
    for (int i = 0; i < trackLength; ++i) {
        track[i] = (i % 2 == 0) ? low : high;
    }
}

void setup() {

    // Return direction by addressing edgeDetectionPin and directionDetectionPin
    pinMode(edgeDetectionPin, INPUT_PULLDOWN);
    pinMode(directionDetectionPin, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(edgeDetectionPin), readDirectionOnRising, RISING);

    // Simulate track (not so important and independent of above)
    createTrack();
    pinMode(driveLeftPin, INPUT_PULLDOWN);
    pinMode(driveRightPin, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(driveLeftPin), driveLeft, CHANGE);
    attachInterrupt(digitalPinToInterrupt(driveRightPin), driveRight, CHANGE);
}


void loop() {
    delay(100);
    Serial.println(direction);
}
