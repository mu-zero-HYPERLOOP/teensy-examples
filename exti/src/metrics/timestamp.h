#pragma once

#include "core_pins.h"
#include "inttypes.h"

class Timestamp {
public:
  [[maybe_unused]] static Timestamp now() { return Timestamp(micros()); }

  [[maybe_unused]] uint32_t as_u32_s() const { return m_us / 1000000; }
  [[maybe_unused]] uint32_t as_u32_ms() const { return m_us / 1000; }
  [[maybe_unused]] uint32_t as_u32_us() const { return m_us; }

  [[maybe_unused]] uint32_t as_s() const { return m_us / 1.e6; }
  [[maybe_unused]] uint32_t as_ms() const { return m_us / 1.e3; }
  [[maybe_unused]] uint32_t as_us() const { return m_us; }

  Timestamp(const Timestamp &) = default;
  Timestamp(const volatile Timestamp &o) : m_us(o.m_us) {}
  Timestamp(Timestamp &&) = default;
  Timestamp(volatile Timestamp &&o) : m_us(o.m_us) {}
  ~Timestamp() = default;
  Timestamp &operator=(const Timestamp &) = default;
  volatile Timestamp &operator=(const volatile Timestamp &o) {
    m_us = o.m_us;
    return *this;
  }
  Timestamp &operator=(Timestamp &&) = default;
  volatile Timestamp &operator=(volatile Timestamp &&o) {
    m_us = o.m_us;
    return *this;
  }

private:
  explicit Timestamp(uint32_t us) : m_us(us) {}
  uint32_t m_us;
};
