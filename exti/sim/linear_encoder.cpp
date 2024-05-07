#include "linear_encoder.h"
#include "timestamp.h"
#include <random>


Timestamp LinearEncoder::last_isr_called;
Distance LinearEncoder::stride;
int32_t LinearEncoder::stripes_counted;
std::normal_distribution<float> LinearEncoder::dist;
std::mt19937 LinearEncoder::generator;
Distance LinearEncoder::stripe_miss[stripe_num];

