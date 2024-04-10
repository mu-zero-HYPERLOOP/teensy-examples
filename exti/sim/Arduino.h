#pragma once

#include "inttypes.h"


#define INPUT_PULLDOWN 1
#define CHANGE 1

extern void pinMode(int, int);


extern void attachInterrupt(int pin, void (*isr)(), int);
extern uint32_t micros();

extern uint8_t digitalReadFast(int pin);

