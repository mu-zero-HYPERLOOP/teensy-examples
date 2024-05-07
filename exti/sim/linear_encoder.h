#pragma once

#include "metrics.h"
#include "timestamp.h"
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <random>
#include "ekf.hpp"
#include <cassert>

constexpr uint8_t stripe_num = 200;

class LinearEncoder {
  private:
    static int32_t stripes_counted;
    static Timestamp last_isr_called;
    static std::mt19937 generator;
    static std::normal_distribution<float> dist;
    static Distance stride;
    static Distance stripe_miss[stripe_num];
  public:
    static void begin(Distance stride) {
      LinearEncoder::stride = stride;
      generator = std::mt19937(std::random_device{}());
      dist = std::normal_distribution(0.0f, 0.003f);
      for (int i = 0; i < stripe_num; i++) {
        stripe_miss[i] = stride * i + std::clamp(Distance(dist(generator)), -5_mm, 5_mm);
      }
    }
    static void set_distance(Distance new_dist, Timestamp timestamp) {
      for (int i = 0; i < stripe_num - 1; i++) {
        if (stripe_miss[i+1] > new_dist && stripe_miss[i] < new_dist) {
          stripes_counted = i;
          last_isr_called = timestamp;
        }
      }
    }

    static int32_t stripe_count() {
      return stripes_counted;
    }
    static Distance position() {
      return stripes_counted * stride;
    }
    static Timestamp last_isr() {
      return Timestamp(last_isr_called.m_time_us);
    }
};
