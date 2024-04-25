#include "linear_encoder.h"
#include "timestamp.h"


Timestamp LinearEncoder::last_isr_called;
Distance LinearEncoder::stride;
uint32_t LinearEncoder::stripes_counted;


