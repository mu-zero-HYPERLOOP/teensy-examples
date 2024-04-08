#pragma once

#include "inttypes.h"

class Resistance {
public:
  [[maybe_unused]] static Resistance from_u32_o(uint32_t o) {
    return Resistance(o * 1000000);
  }
  [[maybe_unused]] static Resistance from_u32_mo(uint32_t mo) {
    return Resistance(mo * 1000);
  }
  [[maybe_unused]] static Resistance from_u32_uo(uint32_t uo) {
    return Resistance(uo);
  }

  [[maybe_unused]] static Resistance from_o(float o) {
    return Resistance(o * 1.e6);
  }
  [[maybe_unused]] static Resistance from_mo(float mo) {
    return Resistance(mo * 1.e3);
  }
  [[maybe_unused]] static Resistance from_uo(float uo) {
    return Resistance(uo);
  }

  [[maybe_unused]] uint32_t as_u32_o() const { return m_uo / 1000000; }
  [[maybe_unused]] uint32_t as_u32_mo() const { return m_uo / 1000; }
  [[maybe_unused]] uint32_t as_u32_uo() const { return m_uo; }
  [[maybe_unused]] float as_o() const { return m_uo / 1.e6; }
  [[maybe_unused]] float as_mo() const { return m_uo / 1.e3; }
  [[maybe_unused]] float as_uo() const { return m_uo; }

  Resistance(const Resistance &) = default;
  Resistance(Resistance &&) = default;
  ~Resistance() = default;
  Resistance &operator=(const Resistance &) = default;
  Resistance &operator=(Resistance &&) = default;

  [[maybe_unused]] Resistance &operator*=(const uint32_t &scalar) {
    m_uo *= scalar;
    return *this;
  }

  [[maybe_unused]] Resistance &operator*=(const float &scalar) {
    m_uo *= scalar;
    return *this;
  }

  [[maybe_unused]] Resistance &operator/=(const uint32_t &scalar) {
    m_uo /= scalar;
    return *this;
  }

  [[maybe_unused]] Resistance &operator/=(const float &scalar) {
    m_uo /= scalar;
    return *this;
  }

  [[maybe_unused]] Resistance &operator+=(const Resistance &a) {
    m_uo += a.m_uo;
    return *this;
  }

  [[maybe_unused]] Resistance &operator-=(const Resistance &a) {
    m_uo -= a.m_uo;
    return *this;
  }

private:
  explicit Resistance(uint32_t uo) : m_uo(uo) {}
  uint32_t m_uo;
};

[[maybe_unused]] static Resistance operator*(const Resistance &a,
                                             const uint32_t &b) {
  return Resistance::from_u32_uo(a.as_u32_uo() * b);
}

[[maybe_unused]] static Resistance operator*(const uint32_t &a,
                                             const Resistance &b) {
  return b * a;
}

[[maybe_unused]] static Resistance operator*(const Resistance &a,
                                             const float &b) {
  return Resistance::from_u32_uo(a.as_u32_uo() * b);
}

[[maybe_unused]] static Resistance operator*(const float &a,
                                             const Resistance &b) {
  return b * a;
}

[[maybe_unused]] static Resistance operator/(const Resistance &a,
                                             const uint32_t &b) {
  return Resistance::from_u32_uo(a.as_u32_uo() / b);
}

[[maybe_unused]] static Resistance operator/(const Resistance &a,
                                             const float &b) {
  return Resistance::from_u32_uo(a.as_u32_uo() / b);
}

[[maybe_unused]] static Resistance operator+(const Resistance &a,
                                             const Resistance &b) {
  return Resistance::from_u32_uo(a.as_u32_uo() + b.as_u32_uo());
}

[[maybe_unused]] static Resistance operator-(const Resistance &a,
                                             const Resistance &b) {
  return Resistance::from_u32_uo(a.as_u32_uo() - b.as_u32_uo());
}
