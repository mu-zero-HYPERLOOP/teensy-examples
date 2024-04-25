#pragma once


#include "metrics.h"
#include "timestamp.h"
#include <cstdint>


class LinearEncoder {
  private:
    static Distance distance;
    static uint32_t stripes_counted;
  public:
    static Distance stride;
    static Timestamp last_isr_called;
    static void set_distance(Distance new_dist, Timestamp timestamp) {
      if (static_cast<uint32_t>(std::floor(new_dist / stride) != stripes_counted)) {
        stripes_counted = static_cast<uint32_t>(std::floor(new_dist / stride));
        last_isr_called = timestamp;
      }
      distance = new_dist;
    }
    static int32_t stripe_count() {
      return stripes_counted;
    }
    static Distance position() {
      return stripes_counted * stride;
    }
    static Timestamp last_isr() {
      return last_isr_called;
    }
};
