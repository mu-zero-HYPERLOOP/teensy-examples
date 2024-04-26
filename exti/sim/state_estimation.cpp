#include "state_estimation.h"
#include "linear_encoder.h"
#include "accelerometer.hpp"
#include "ekf.hpp"
#include "metrics.h"
#include "timestamp.h"
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>

static int32_t last_stripe_count = 0;
constexpr Frequency ACCELEROMETER_FREQ = 2_Hz;
static Timestamp last_accel;
static Timestamp last_update;
static Distance s_pos;
static Velocity s_vel;
static Acceleration s_acc;
constexpr float stripe_variance = 0.000001f;
constexpr float imu_variance = 0.00001f;
Ekf<DIM_STATE, DIM_OBSER>StateEstimation::ekf;

void StateEstimation::begin() {
  last_accel = Timestamp::now();

  for (int i = 0; i < DIM_STATE; i++) {
    ekf.x_hat[i] = 1.0f; // arbitrary start. must not be zero!
  }
  for (int i = 0; i < DIM_STATE; i++) {
    for (int j = 0; j < DIM_STATE; j++) {
      ekf.P_pre[i * DIM_STATE + j] = 0.0f;
      ekf.F[i * DIM_STATE + j] = 0.0f;
      ekf.F_T[i * DIM_STATE + j] = 0.0f;
      ekf.Q[i * DIM_STATE + j] = 0.0; // process noise
    }
    ekf.P_pre[i * DIM_STATE + i] = 1.0f;
    ekf.F[i * DIM_STATE + i] = 1.0f;
    ekf.F_T[i * DIM_STATE + i] = 1.0f;
  }
  for (int i = 0; i < DIM_OBSER; i++) {
    for (int j = 0; j < DIM_OBSER; j++) {
      ekf.R[i * DIM_OBSER + j] = 0.0f;
    }
  }
  ekf.R[stripe_i * DIM_OBSER + stripe_i] = stripe_variance; // variance of feducial
  ekf.R[imu_i * DIM_OBSER + imu_i] = imu_variance; // variance of imu

  for (int i = 0; i < DIM_OBSER; i++) {
    for (int j = 0; j < DIM_STATE; j++) {
      ekf.H[i * DIM_STATE + j] = 0.0f;
      ekf.H_T[j * DIM_OBSER + i] = 0.0f;
    }
  }
  ekf.H[0 * DIM_STATE + 0] = 1.0f;
  ekf.H[1 * DIM_STATE + 2] = 1.0f;
  ekf.H_T[0 * DIM_OBSER + 0] = 1.0f;
  ekf.H_T[2 * DIM_OBSER + 1] = 1.0f;
  last_update = Timestamp::now();
}

void StateEstimation::update() {
  uint32_t new_stripe_count = LinearEncoder::stripe_count();
  if (last_stripe_count != new_stripe_count) {
    last_stripe_count = new_stripe_count;
    StateEstimation::position_update(LinearEncoder::position(), LinearEncoder::last_isr());
  }
  const Timestamp now = Timestamp::now();
  if (now - last_accel > 1.0f / ACCELEROMETER_FREQ) {
    Acceleration accel = Accelerometer::readAccel();
    acceleration_update(accel, now);
    last_accel = last_accel + 1.0f / ACCELEROMETER_FREQ;
  }
}
Distance StateEstimation::getPosition() {
  return Distance(ekf.x_hat[pos_i]);
}
Velocity StateEstimation::getVelocity() {
  return Velocity(ekf.x_hat[speed_i]);
}
Acceleration StateEstimation::getAcceleration() {
  return Acceleration(ekf.x_hat[acc_i]);
}


void StateEstimation::position_update(const Distance &pos,
                                      const Timestamp &timestamp) {
  const float dur_us = (timestamp - last_update).as_us();
  //std::printf("duration since last update in us: %f\n", dur_us);
  last_update = Timestamp::now();
  constexpr float us_in_s = 1000000.0f;
  // predict new state based on old one
  ekf.f_xu[pos_i] = ekf.x_hat[pos_i] 
    + dur_us * ekf.x_hat[speed_i] / us_in_s 
    + 0.5 * dur_us * dur_us * ekf.x_hat[acc_i] / us_in_s / us_in_s;
  ekf.f_xu[speed_i] = ekf.x_hat[speed_i] + dur_us * ekf.x_hat[acc_i] / us_in_s;
  ekf.f_xu[acc_i] = ekf.x_hat[acc_i];

  // set jacobian of process matrix
  ekf.F[0 * DIM_STATE + 1] = dur_us / us_in_s;
  ekf.F[0 * DIM_STATE + 2] = 0.5f * dur_us * dur_us / us_in_s / us_in_s;
  ekf.F[1 * DIM_STATE + 2] = dur_us / us_in_s;
  ekf.F_T[1 * DIM_STATE + 0] = dur_us / us_in_s;
  ekf.F_T[2 * DIM_STATE + 0] = 0.5f * dur_us * dur_us / us_in_s / us_in_s;
  ekf.F_T[2 * DIM_STATE + 1] = dur_us / us_in_s;

  // set expected measurements, H is constant and does not have to be changed
  ekf.h_x[stripe_i] = ekf.f_xu[pos_i];
  ekf.h_x[imu_i] = ekf.f_xu[acc_i];

  float measurement[DIM_OBSER];
  measurement[stripe_i] = static_cast<float>(pos);
  //std::printf("pos: %f\n", measurement[stripe_i]);
  measurement[imu_i] = ekf.f_xu[acc_i]; // use predicted value as missing measurement
  //ekf.R[imu_i * DIM_OBSER + imu_i] = std::numeric_limits<float>::max(); // and its variance high 
                                              // => measurement should be ignored by filter
  ekf.R[imu_i * DIM_OBSER + imu_i] = 100000.0f;
  int failure = ekf_step<DIM_STATE, DIM_OBSER>(ekf, measurement);
  //std::printf("failure: %d\n", failure);
  ekf.R[imu_i * DIM_OBSER + imu_i] = imu_variance;
}

void StateEstimation::acceleration_update(const Acceleration &acc,
                                          const Timestamp &timestamp) {
  const float dur_us = (timestamp - last_update).as_us();
  last_update = Timestamp::now();
  constexpr float us_in_s = 1000000.0f;
  // predict new state based on old one
  ekf.f_xu[pos_i] = ekf.x_hat[pos_i] 
    + dur_us * ekf.x_hat[speed_i] / us_in_s 
    + 0.5 * dur_us * dur_us * ekf.x_hat[acc_i] / us_in_s / us_in_s;
  ekf.f_xu[speed_i] = ekf.x_hat[speed_i] + dur_us * ekf.x_hat[acc_i] / us_in_s;
  ekf.f_xu[acc_i] = ekf.x_hat[acc_i];

  // set jacobian of process matrix
  ekf.F[0 * DIM_STATE + 1] = dur_us / us_in_s;
  ekf.F[0 * DIM_STATE + 2] = 0.5f * dur_us * dur_us / us_in_s / us_in_s;
  ekf.F[1 * DIM_STATE + 2] = dur_us / us_in_s;
  ekf.F_T[1 * DIM_STATE + 0] = dur_us / us_in_s;
  ekf.F_T[2 * DIM_STATE + 0] = 0.5f * dur_us * dur_us / us_in_s / us_in_s;
  ekf.F_T[2 * DIM_STATE + 1] = dur_us / us_in_s;

  // set expected measurements, H is constant and does not have to be changed
  ekf.h_x[stripe_i] = ekf.f_xu[pos_i];
  ekf.h_x[imu_i] = ekf.f_xu[acc_i];

  float measurement[DIM_OBSER];
  measurement[stripe_i] = ekf.f_xu[pos_i]; // use predicted value as missing measurement
  measurement[imu_i] = static_cast<float>(acc); 
  //ekf.R[stripe_i * DIM_OBSER + stripe_i] = std::numeric_limits<float>::infinity(); // and its variance high 
                                              // => measurement should be ignored by filter
  ekf.R[stripe_i * DIM_OBSER + stripe_i] = 100000.0f;
  ekf_step<DIM_STATE, DIM_OBSER>(ekf, measurement);
  ekf.R[stripe_i * DIM_OBSER + stripe_i] = stripe_variance;
}
