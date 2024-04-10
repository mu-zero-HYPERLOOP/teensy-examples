#pragma once

#include "distance.h"
#include "duration.h"
#include "timestamp.h"
#include "velocity.h"

#include "current.h"
#include "resistance.h"
#include "voltage.h"

#include "inttypes.h"

// basic Kinematic

// v = d / t
[[maybe_unused]] static Velocity operator/(const Distance &dis,
                                           const Duration &dur) {
  assert(dur.as_u32_us() != 0);
  return Velocity::from_i32_um_per_s((((uint64_t)dis.as_u32_um()) * 1000000ull) /
                                     (uint64_t)dur.as_u32_us());
}
// d = v * t
[[maybe_unused]] static Distance operator*(const Velocity &vel,
                                           const Duration &dur) {
  return Distance::from_u32_um(vel.as_i32_um_per_s() * dur.as_u32_us());
}
[[maybe_unused]] static Distance operator*(const Duration &dur,
                                           const Velocity &vel) {
  return vel * dur;
}

// t = d / v
[[maybe_unused]] static Duration operator/(const Duration &dur,
                                           const Velocity &vel) {
  int32_t vel_abs_um_per_s = vel.as_i32_um_per_s();
  vel_abs_um_per_s =
      vel_abs_um_per_s >= 0 ? vel_abs_um_per_s : -vel_abs_um_per_s; // abs(vel);
  return Duration::from_u32_us((((uint64_t)dur.as_u32_us()) * 1000000) /
                               (uint64_t)vel_abs_um_per_s);
}

// Ohms law

// u = r * i
[[maybe_unused]] static Voltage operator*(const Resistance &r,
                                          const Current &i) {
  return Voltage::from_i32_uv(r.as_u32_uo() * i.as_i32_ua());
}
[[maybe_unused]]
[[maybe_unused]] static Voltage operator*(const Current &i,
                                          const Resistance &r) {
  return Voltage::from_i32_uv(r.as_u32_uo() * i.as_i32_ua());
}

// r = u / i
[[maybe_unused]] static Resistance operator/(const Voltage &v,
                                             const Current &i) {
  assert(i.as_i32_ua() != 0);
  int64_t uo =
      ((((uint64_t)v.as_i32_uv()) * 1000000) / ((uint64_t)i.as_i32_ua()));
  assert(uo >= 0);
  return Resistance::from_u32_uo(uo);
}

// i = u / r
[[maybe_unused]] static Current operator/(const Voltage &v,
                                          const Resistance &r) {
  assert(r.as_u32_uo() != 0);
  return Current::from_i32_ua((((uint64_t)v.as_i32_uv()) * 1000000) /
                              ((uint64_t)r.as_u32_uo()));
}


// utilities

static Distance operator+(const Distance& a, const Velocity& b) {
  return Distance::from_u32_um(a.as_u32_um() + b.as_i32_um_per_s());
}

