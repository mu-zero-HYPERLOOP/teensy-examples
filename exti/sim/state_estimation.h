#pragma once
#include "metrics.h"
#include "timestamp.h"
#include <cstdint>
#include "ekf.hpp"

constexpr uint8_t DIM_STATE = 3;
constexpr uint8_t DIM_OBSER = 2;

constexpr unsigned int pos_i = 0;
constexpr unsigned int speed_i = 1;
constexpr unsigned int acc_i = 2;
constexpr unsigned int stripe_i = 0;
constexpr unsigned int imu_i = 1;



class StateEstimation {
public:
  static void begin();
  static void update();
  static Distance getPosition();
  static Velocity getVelocity();
  static Acceleration getAcceleration();

private:
  StateEstimation() = delete;
  static void position_update(const Distance& pos, const Timestamp& timstamp);
  static void acceleration_update(const Acceleration& acc, const Timestamp& timestamp);
  using EKF = Ekf<DIM_STATE, DIM_OBSER>;
  static EKF ekf;
};

