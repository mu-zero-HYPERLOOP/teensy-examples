#include "imxrt.h"
#include <Arduino.h>

void setup() {
  PORTD |= 1;
  FLEXPWM1_SM0INIT &= 0; // initial count set to 0


}

void loop() {

}
