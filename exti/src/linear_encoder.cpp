#include "linear_encoder.h"

/**
 * Some interessting notes here
 * the principle is really simple
 * we have 2 pins, where we attach a interrupt to one
 * when the interrupt is triggered we perform
 * left == right if this is true we are moving to the right otherwise we
 * are moving to the left.
 *
 * Now immagine, we are not moving at all, but the sensors are placed on stop of
 * a edge. this will leads to a lot of interrupts. e.g, moving left, moving
 * right, moving left moving right. For position estimation this would kind of
 * suck. Therefor it is designed with the following invariant. If we cross a
 * stripe edge from the left. we can assume that we are right from the current
 * stripe count value and if we cross it from the right we are at the left side
 * of the stripe count value.
 *
 * This is implemented by m_dir. Here we only change m_stripe_count if we move
 * at least 2 times in one direction. Therefor move left, move-right, move-left
 * would not be reflected in the stripe_count value!
 */

int LinearEncoder::m_left_pin;
int LinearEncoder::m_right_pin;
volatile uint32_t LinearEncoder::m_stripe_count;
volatile LinearEncoder::Direction LinearEncoder::m_dir;
volatile uint32_t LinearEncoder::m_last_isr_us;
volatile uint32_t LinearEncoder::m_velocity_um_per_s;
Distance LinearEncoder::m_stride = Distance::from_u32_um(0);

void LinearEncoder::begin(const LinearEncoderBeginInfo &beginInfo) {
  m_left_pin = beginInfo.left_pin;
  m_right_pin = beginInfo.right_pin;
  m_stride = beginInfo.stride;

  pinMode(m_left_pin, INPUT_PULLDOWN);
  attachInterrupt(m_left_pin, LinearEncoder::encoder_isr, CHANGE);
  m_stripe_count = 0;
  m_dir = DIRECTION_NONE;
  m_last_isr_us = micros();
}

uint16_t LinearEncoder::stripe_count() { return m_stripe_count; }

void LinearEncoder::reset(uint16_t stripe_count) {
  m_stripe_count = stripe_count;
  m_dir = DIRECTION_NONE;
  m_velocity_um_per_s = 0;
  m_last_isr_us = micros();
}

void LinearEncoder::encoder_isr() {
  uint8_t left = digitalReadFast(m_left_pin);
  uint8_t right = digitalReadFast(m_right_pin);
  enum Direction dir = static_cast<enum Direction>(left == right);
  /* assert(dir != DIRECTION_NONE); */

  uint32_t now = micros();
  Duration delta = Duration::from_u32_us(now - m_last_isr_us);
  m_last_isr_us = delta.as_u32_us();

  if (dir == m_dir) {
    if (dir == DIRECTION_LEFT) {
      m_stripe_count -= 1;
      m_velocity_um_per_s = (m_stride / delta).as_um_per_s();
    } else {
      /* assert(dir == DIRECTION_RIGHT); */
      m_stripe_count += 1;
      m_velocity_um_per_s = -(m_stride / delta).as_um_per_s();
    }
    m_stripe_count += dir ? 1 : -1;
  } else {
    m_velocity_um_per_s = 0;
  }
  m_dir = dir;
}
