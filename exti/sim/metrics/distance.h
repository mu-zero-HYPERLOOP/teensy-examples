#pragma once

#include "inttypes.h"
#include <cassert>

class Distance {
public:
  [[maybe_unused]] static Distance from_u32_um(uint32_t um) {
    return Distance(um);
  }
  [[maybe_unused]] static Distance from_u32_mm(uint32_t mm) {
    return Distance(mm * 1000);
  }
  [[maybe_unused]] static Distance from_u32_m(uint32_t m) {
    return Distance(m * 1000000);
  }
  [[maybe_unused]] static Distance from_um(float um) { return Distance(um); }
  [[maybe_unused]] static Distance from_mm(float mm) {
    return Distance(mm * 1.e3);
  }
  [[maybe_unused]] static Distance from_m(float m) {
    return Distance(m * 1.e6);
  }

  [[maybe_unused]] uint32_t as_u32_um() const { return m_um; }
  /// NOTE lossy convertion
  [[maybe_unused]] uint32_t as_u32_mm() const { return m_um / 1000; }
  /// NOTE lossy convertion
  [[maybe_unused]] uint32_t as_u32_m() const { return m_um / 1000000; }
  [[maybe_unused]] float as_um() const { return m_um; }
  [[maybe_unused]] float as_mm() const { return m_um / 1.e3; }
  [[maybe_unused]] float as_m() const { return m_um / 1.e6; }

  Distance(const Distance &) = default;
  Distance(Distance &&) = default;
  ~Distance() = default;
  Distance &operator=(const Distance &) = default;
  Distance &operator=(Distance &&) = default;

  [[maybe_unused]] Distance &operator*=(const uint32_t &scalar) {
    m_um *= scalar;
    return *this;
  }

  [[maybe_unused]] Distance &operator*=(const float &scalar) {
    m_um *= scalar;
    return *this;
  }

  [[maybe_unused]] Distance &operator/=(const uint32_t &scalar) {
    m_um /= scalar;
    return *this;
  }

  [[maybe_unused]] Distance &operator/=(const float &scalar) {
    m_um /= scalar;
    return *this;
  }

  [[maybe_unused]] Distance &operator+=(const Distance &a) {
    m_um += a.m_um;
    return *this;
  }

  [[maybe_unused]] Distance &operator-=(const Distance &a) {
    m_um -= a.m_um;
    return *this;
  }

private:
  inline explicit Distance(uint32_t um) : m_um(um) {}
  uint32_t m_um;
};

[[maybe_unused]] static Distance operator*(const Distance &a,
                                           const uint32_t &b) {
  return Distance::from_u32_um(a.as_u32_um() * b);
}

[[maybe_unused]] static Distance operator*(const uint32_t &a,
                                           const Distance &b) {
  return b * a;
}

[[maybe_unused]] static Distance operator*(const Distance &a, const float &b) {
  return Distance::from_u32_um(a.as_u32_um() * b);
}

[[maybe_unused]] static Distance operator*(const float &a, const Distance &b) {
  return b * a;
}

[[maybe_unused]] static Distance operator/(const Distance &a,
                                           const uint32_t &b) {
  return Distance::from_u32_um(a.as_u32_um() / b);
}

[[maybe_unused]] static Distance operator/(const Distance &a, const float &b) {
  return Distance::from_u32_um(a.as_u32_um() / b);
}

[[maybe_unused]] static Distance operator+(const Distance &a,
                                           const Distance &b) {
  return Distance::from_u32_um(a.as_u32_um() + b.as_u32_um());
}

[[maybe_unused]] static Distance operator-(const Distance &a,
                                           const Distance &b) {
  assert(a.as_u32_um() > b.as_u32_um());
  return Distance::from_u32_um(a.as_u32_um() - b.as_u32_um());
}
