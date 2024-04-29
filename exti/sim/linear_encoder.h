#pragma once

#include "metrics.h"
#include "timestamp.h"
#include <cstdint>
#include <cstdio>
#include <random>
#include "ekf.hpp"
#include <cassert>


class LinearEncoder {
  private:
    static int32_t stripes_counted;
    static Timestamp last_isr_called;
    static std::mt19937 generator;
    static std::normal_distribution<float> dist;
    static Distance stride;
  public:
    static void begin(Distance stride, float mean, float deviation) {
      LinearEncoder::stride = stride;
      generator = std::mt19937(std::random_device{}());
      dist = std::normal_distribution(mean, deviation);
    }
    static void set_distance(Distance new_dist, Timestamp timestamp) {
      Distance measured_distance = new_dist + Distance(dist(generator));
      int32_t measurement = static_cast<uint32_t>(
          std::floor(static_cast<float>(measured_distance / stride)));
      if (print) std::printf("linear encoder stripes: %d and dist: %f\n", measurement, static_cast<float>(measurement * stride));
      if (measurement != stripes_counted) {
        stripes_counted = measurement;
        last_isr_called = timestamp;
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
