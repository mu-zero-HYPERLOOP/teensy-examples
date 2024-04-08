#pragma once

#include "inttypes.h"

class Current {
public:

[[maybe_unused]]
  static Current from_i32_a(int32_t a) { return Current(a * 1000000); }
[[maybe_unused]]
  static Current from_i32_ma(int32_t ma) { return Current(ma * 1000); }
[[maybe_unused]]
  static Current from_i32_ua(int32_t ua) { return Current(ua); }
[[maybe_unused]]
  static Current from_a(float a) { return Current(a * 1.e6); }
[[maybe_unused]]
  static Current from_ma(float ma) { return Current(ma * 1.e3); }
[[maybe_unused]]
  static Current from_ua(float ua) { return Current(ua); }

[[maybe_unused]]
  int32_t as_i32_a() const { return m_ua / 1000000; }
[[maybe_unused]]
  int32_t as_i32_ma() const { return m_ua / 1000; }
[[maybe_unused]]
  uint32_t as_i32_ua() const { return m_ua; }
[[maybe_unused]]
  float as_a() const { return m_ua / 1.e6; }
[[maybe_unused]]
  float as_ma() const { return m_ua / 1.e3; }
[[maybe_unused]]
  float as_ua() const { return m_ua; }

  Current(const Current &) = default;
  Current(Current &&) = default;
  ~Current() = default;
  Current &operator=(const Current &) = default;
  Current &operator=(Current &&) = default;

[[maybe_unused]]
  Current &operator*=(const int32_t &scalar) {
    m_ua *= scalar;
    return *this;
  }

[[maybe_unused]]
  Current &operator*=(const float &scalar) {
    m_ua *= scalar;
    return *this;
  }

[[maybe_unused]]
  Current &operator/=(const int32_t &scalar) {
    m_ua /= scalar;
    return *this;
  }

[[maybe_unused]]
  Current &operator/=(const float &scalar) {
    m_ua /= scalar;
    return *this;
  }

[[maybe_unused]]
  Current &operator+=(const Current &a) {
    m_ua += a.as_i32_ua();
    return *this;
  }

[[maybe_unused]]
  Current &operator-=(const Current &a) {
    m_ua -= a.as_i32_ua();
    return *this;
  }

private:
  Current(int32_t ua) : m_ua(ua) {}
  int32_t m_ua;
};

[[maybe_unused]]
static Current operator*(const Current &a, const uint32_t &b) {
  return Current::from_i32_ua(a.as_i32_ua() * b);
}

[[maybe_unused]]
static Current operator*(const uint32_t &a, const Current &b) { return b * a; }

[[maybe_unused]]
static Current operator*(const Current &a, const float &b) {
  return Current::from_i32_ua(a.as_i32_ua() * b);
}

[[maybe_unused]]
static Current operator*(const float &a, const Current &b) { return b * a; }

[[maybe_unused]]
static Current operator/(const Current &a, const uint32_t &b) {
  return Current::from_i32_ua(a.as_i32_ua() / b);
}

[[maybe_unused]]
static Current operator/(const Current &a, const float &b) {
  return Current::from_i32_ua(a.as_i32_ua() / b);
}

[[maybe_unused]]
static Current operator+(const Current &a, const Current &b) {
  return Current::from_i32_ua(a.as_i32_ua() + b.as_i32_ua());
}

[[maybe_unused]]
static Current operator-(const Current &a, const Current &b) {
  return Current::from_i32_ua(a.as_i32_ua() - b.as_i32_ua());
}
