#pragma once

#include "inttypes.h"
#include "timestamp.h"

class Duration {
public:
  [[maybe_unused]] static Duration from_u32_s(uint32_t s) {
    return Duration(s * 1000000);
  }
  [[maybe_unused]] static Duration from_u32_ms(uint32_t ms) {
    return Duration(ms * 1000);
  }
  [[maybe_unused]] static Duration from_u32_us(uint32_t us) {
    return Duration(us);
  }

  [[maybe_unused]] static Duration from_s(float s) {
    return Duration(s * 1.e6);
  }
  [[maybe_unused]] static Duration from_ms(float ms) {
    return Duration(ms * 1.e3);
  }
  [[maybe_unused]] static Duration from_us(float us) { return Duration(us); }

  [[maybe_unused]] static Duration since(const Timestamp &other) {
    return Duration::from_u32_us(Timestamp::now().as_u32_us() -
                                 other.as_u32_us());
  }

  [[maybe_unused]] uint32_t as_u32_s() const { return m_us / 1000000; }
  [[maybe_unused]] uint32_t as_u32_ms() const { return m_us / 1000; }
  [[maybe_unused]] uint32_t as_u32_us() const { return m_us; }
  [[maybe_unused]] float as_s() const { return m_us / 1.e6; }
  [[maybe_unused]] float as_ms() const { return m_us / 1.e3; }
  [[maybe_unused]] float as_us() const { return m_us; }

  Duration(const Duration &) = default;
  Duration(Duration &&) = default;
  ~Duration() = default;
  Duration &operator=(const Duration &) = default;
  Duration &operator=(Duration &&) = default;

  [[maybe_unused]] Duration &operator*=(const uint32_t &scalar) {
    m_us *= scalar;
    return *this;
  }

  [[maybe_unused]] Duration &operator*=(const float &scalar) {
    m_us *= scalar;
    return *this;
  }

  [[maybe_unused]] Duration &operator/=(const uint32_t &scalar) {
    m_us /= scalar;
    return *this;
  }

  [[maybe_unused]] Duration &operator/=(const float &scalar) {
    m_us /= scalar;
    return *this;
  }

  [[maybe_unused]] Duration &operator+=(const Duration &a) {
    m_us += a.m_us;
    return *this;
  }

  [[maybe_unused]] Duration &operator-=(const Duration &a) {
    m_us -= a.m_us;
    return *this;
  }

private:
  Duration(uint32_t us) : m_us(us) {}
  uint32_t m_us;
};

[[maybe_unused]] static Duration operator*(const Duration &a,
                                           const uint32_t &b) {
  return Duration::from_u32_us(a.as_us() * b);
}

[[maybe_unused]] static Duration operator*(const uint32_t &a,
                                           const Duration &b) {
  return b * a;
}

[[maybe_unused]] static Duration operator*(const Duration &a, const float &b) {
  return Duration::from_u32_us(a.as_us() * b);
}

[[maybe_unused]] static Duration operator*(const float &a, const Duration &b) {
  return b * a;
}

[[maybe_unused]] static Duration operator/(const Duration &a,
                                           const uint32_t &b) {
  return Duration::from_u32_us(a.as_us() / b);
}

[[maybe_unused]] static Duration operator/(const Duration &a, const float &b) {
  return Duration::from_u32_us(a.as_us() / b);
}

[[maybe_unused]] static Duration operator+(const Duration &a,
                                           const Duration &b) {
  return Duration::from_u32_us(a.as_us() + b.as_us());
}

[[maybe_unused]] static Duration operator-(const Duration &a,
                                           const Duration &b) {
  return Duration::from_u32_us(a.as_us() - b.as_us());
}

[[maybe_unused]] static Duration operator-(const Timestamp &a,
                                           const Timestamp &b) {
  return Duration::from_u32_us(a.as_u32_us() - b.as_u32_us());
}
