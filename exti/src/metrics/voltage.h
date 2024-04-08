#pragma once

#include "inttypes.h"

class Voltage {
public:
  [[maybe_unused]] static Voltage from_i32_v(uint32_t v) {
    return Voltage(v * 1000000);
  }
  [[maybe_unused]] static Voltage from_i32_mv(uint32_t mv) {
    return Voltage(mv * 1000);
  }
  [[maybe_unused]] static Voltage from_i32_uv(uint32_t uv) {
    return Voltage(uv);
  }
  [[maybe_unused]] static Voltage from_v(float v) { return Voltage(v * 1.e6); }
  [[maybe_unused]] static Voltage from_mv(float mv) {
    return Voltage(mv * 1.e3);
  }
  [[maybe_unused]] static Voltage from_uv(float uv) { return Voltage(uv); }

  [[maybe_unused]] int32_t as_i32_v() const { return m_uv / 1000000; }
  [[maybe_unused]] int32_t as_i32_mv() const { return m_uv / 1000; }
  [[maybe_unused]] uint32_t as_i32_uv() const { return m_uv; }
  [[maybe_unused]] float as_v() const { return m_uv / 1.e6; }
  [[maybe_unused]] float as_mv() const { return m_uv / 1.e3; }
  [[maybe_unused]] float as_uv() const { return m_uv; }

  Voltage(const Voltage &) = default;
  Voltage(Voltage &&) = default;
  ~Voltage() = default;
  Voltage &operator=(const Voltage &) = default;
  Voltage &operator=(Voltage &&) = default;

private:
  Voltage(uint32_t uv) : m_uv(uv){};
  int32_t m_uv;
};
