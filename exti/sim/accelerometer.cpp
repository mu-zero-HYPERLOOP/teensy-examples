#include "accelerometer.hpp"
#include <random>

Acceleration Accelerometer::acceleration;
std::mt19937 Accelerometer::generator;
std::normal_distribution<float> Accelerometer::dist;
