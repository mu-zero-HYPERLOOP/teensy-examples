#pragma once


#include "Arduino.h"
#include "metrics/metrics.h"


struct LinearEncoderBeginInfo {
  int left_pin;
  int right_pin;
  Distance stride = Distance::from_u32_um(0);
};

struct LinearEncoder {
private:
  enum Direction : uint8_t {
    DIRECTION_LEFT = 0,
    DIRECTION_RIGHT = 1,
    DIRECTION_NONE = 2,
  };

public:
  static void begin(const LinearEncoderBeginInfo &beginInfo);

  static void reset(uint16_t stripe_count = 0);

  static uint16_t stripe_count();

  static Distance distance() { return m_stride * m_stripe_count; }

  static Velocity velocity() { 
    if (Timestamp::now().as_u32_us() - m_last_isr_us > 1000000) {
      return Velocity::from_i32_um_per_s(0);
    }else {
      return Velocity::from_i32_um_per_s(m_velocity_um_per_s); 
    }
  }

  static Direction direction() { return m_dir; }

private:
  LinearEncoder() {}
  static void encoder_isr();

  static int m_left_pin;
  static volatile uint32_t m_stripe_count;
  static int m_right_pin;
  static volatile enum Direction m_dir;
  static volatile uint32_t m_last_isr_us;
  static volatile uint32_t m_velocity_um_per_s;
  static Distance m_stride;
};
