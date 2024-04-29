#include "metrics.h"
#include <random>

class Accelerometer {
  private:
    static std::mt19937 generator;
    static std::normal_distribution<float> dist;
  public:
    static Acceleration acceleration;
    static void begin(float mean, float deviation) {
      generator = std::mt19937(std::random_device{}());
      dist = std::normal_distribution(mean,deviation);
    }
    static Acceleration readAccel() {
      Acceleration noisyAccel = Acceleration(static_cast<float>(Accelerometer::acceleration));
      noisyAccel += Acceleration(dist(generator));
      return noisyAccel;
    }
};


