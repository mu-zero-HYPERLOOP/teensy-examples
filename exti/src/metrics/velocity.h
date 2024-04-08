#pragma once

#include "inttypes.h"

class Velocity {
public:
  [[maybe_unused]] static Velocity from_i32_m_per_s(int32_t m_per_s) {
    return Velocity(m_per_s * 1000000);
  }
  [[maybe_unused]] static Velocity from_i32_mm_per_s(int32_t mm_per_s) {
    return Velocity(mm_per_s * 1000);
  }
  /// NOTE lossy convertion
  [[maybe_unused]] static Velocity from_i32_um_per_s(int32_t um_per_s) {
    return Velocity(um_per_s);
  }

  [[maybe_unused]] static Velocity from_m_per_s(float m_per_s) {
    return Velocity(m_per_s * 1.e6);
  }
  [[maybe_unused]] static Velocity from_mm_per_s(float mm_per_s) {
    return Velocity(mm_per_s * 1.e3);
  }
  /// NOTE lossy convertion
  [[maybe_unused]] static Velocity from_um_per_s(float um_per_s) {
    return Velocity(um_per_s);
  }

  [[maybe_unused]] int32_t as_i32_m_per_s() const {
    return m_um_per_s / 1000000;
  }
  [[maybe_unused]] int32_t as_i32_mm_per_s() const { return m_um_per_s / 1000; }
  [[maybe_unused]] int32_t as_i32_um_per_s() const { return m_um_per_s; }

  [[maybe_unused]] float as_m_per_s() const { return m_um_per_s / 1.e6; }
  [[maybe_unused]] float as_mm_per_s() const { return m_um_per_s / 1.e3; }
  [[maybe_unused]] float as_um_per_s() const { return m_um_per_s; }

  Velocity(const Velocity &other) = default;

  Velocity(const volatile Velocity &other) : m_um_per_s(other.m_um_per_s) {}

  Velocity(Velocity &&other) = default;

  Velocity(volatile Velocity &&other) : m_um_per_s(other.m_um_per_s) {}

  ~Velocity() = default;

  Velocity &operator=(const Velocity &other) = default;

  volatile Velocity &operator=(const volatile Velocity &other) {
    m_um_per_s = other.m_um_per_s;
    return *this;
  }
  Velocity &operator=(Velocity &&other) = default;

  volatile Velocity &operator=(volatile Velocity &&other) {
    m_um_per_s = other.m_um_per_s;
    return *this;
  }

  [[maybe_unused]] Velocity &operator*=(const uint32_t &a) {
    m_um_per_s *= a;
    return *this;
  }

  [[maybe_unused]] Velocity &operator*=(const float &a) {
    m_um_per_s *= a;
    return *this;
  }

  [[maybe_unused]] Velocity &operator+=(const Velocity &a) {
    m_um_per_s += a.m_um_per_s;
    return *this;
  }

  [[maybe_unused]] Velocity &operator-=(const Velocity &a) {
    m_um_per_s -= a.m_um_per_s;
    return *this;
  }

private:
  inline explicit Velocity(uint32_t mm_per_s) : m_um_per_s(mm_per_s) {}
  int32_t m_um_per_s;
};

[[maybe_unused]] static Velocity operator*(const Velocity &a,
                                           const uint32_t &b) {
  return Velocity::from_i32_um_per_s(a.as_i32_um_per_s() * b);
}
[[maybe_unused]] static Velocity operator*(const uint32_t &a,
                                           const Velocity &b) {
  return b * a;
}
[[maybe_unused]] static Velocity operator*(const Velocity &a, float b) {
  return Velocity::from_i32_um_per_s(a.as_i32_um_per_s() * b);
}
[[maybe_unused]] static Velocity operator*(const float &a, const Velocity &b) {
  return b * a;
}
[[maybe_unused]] static Velocity operator+(const Velocity &a,
                                           const Velocity &b) {
  return Velocity::from_i32_um_per_s(a.as_i32_um_per_s() + b.as_i32_um_per_s());
}
[[maybe_unused]] static Velocity operator-(const Velocity &a,
                                           const Velocity &b) {
  return Velocity::from_i32_um_per_s(a.as_i32_um_per_s() - b.as_i32_um_per_s());
}
