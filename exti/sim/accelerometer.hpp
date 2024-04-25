#include "metrics.h"

class Accelerometer {
  public:
    static Acceleration acceleration;
    static Acceleration readAccel() {
      Acceleration noisyAccel = Acceleration(static_cast<float>(acceleration) + );
      return noisyAccel;
    }
};


