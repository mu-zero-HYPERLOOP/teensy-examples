#include "linear_encoder.h"
#include "metrics.h"
#include "timestamp.h"
#include <cmath>
#include <cstdio>
#include <iostream>
#include "accelerometer.hpp"
#include "state_estimation.h"
#include "ekf.hpp"

Duration SIM_DUR = Duration(1_s);
Duration STEP = Duration(10_us);
Acceleration ACCEL = Acceleration(1_mps2);
Velocity MAX_VEL = Velocity(3_mps);
Duration DECEL_TIME = Duration(2_s);
Distance STRIDE = Distance(2_cm); 
uint32_t g_micros = 0;

uint32_t micros() {
   return g_micros;
}


void sim1() {
  LinearEncoder::begin(STRIDE);
  StateEstimation::begin();
  Accelerometer::begin(0.0, 0.15);

  Distance true_distance = Distance(0_m);
  Velocity true_vel = Velocity(0_mps);
  std::mt19937 generator = std::mt19937(std::random_device{}());
  std::normal_distribution distr = std::normal_distribution(0.0, 0.05);

  std::cout << "s_read,a_read,time,s_true,v_true,a_true,"
               "s_linenc,s_kalman,v_kalman,a_kalman,s_error\n";

  for (Duration sim_dur = Duration(0_s); sim_dur < SIM_DUR; sim_dur += STEP) {
    g_micros = sim_dur.as_us();
    Acceleration accel = Acceleration(distr(generator));

    if (sim_dur > DECEL_TIME) {
      if (true_vel > 0_mps) {
        accel -= 1_mps2;
      }
    } else {
      if (true_vel < MAX_VEL) {
        accel += 1_mps2;
      }
    }
    true_vel = true_vel + accel * static_cast<Time>(STEP);
    true_distance = true_distance 
      + true_vel * static_cast<Time>(STEP) + 
      0.5 * accel * static_cast<Time>(STEP) * static_cast<Time>(STEP);
    
    LinearEncoder::set_distance(true_distance, Timestamp(sim_dur.as_us()));
    Accelerometer::acceleration = accel;

    StateEstimation::update();

    Distance s_linenc = LinearEncoder::position();
    Distance s_kalman = StateEstimation::getPosition();
    Velocity v_kalman = StateEstimation::getVelocity();
    Acceleration a_kalman = StateEstimation::getAcceleration();
    float distance_error = static_cast<float>(true_distance - s_kalman);
    uint32_t c = LinearEncoder::stripe_count();
    if (!print) {
      std::cout << static_cast<float>(sim_dur.as_us()) / 1000000.0f << "," 
                << static_cast<float>(true_distance) << ","                
                << static_cast<float>(true_vel) << ","                     
                << static_cast<float>(accel) << ","
                << static_cast<float>(s_linenc) << ","                     
                << static_cast<float>(s_kalman) << ","                     
                << static_cast<float>(v_kalman) << ","                     
                << static_cast<float>(a_kalman) << ","                     
                << distance_error <<
                "\n";
      std::cout.flush();
    }
  }
}


void sim2() {
  LinearEncoder::begin(STRIDE);
  StateEstimation::begin();
  Accelerometer::begin(0.0, 0.15);

  Distance true_distance = Distance(0_m);
  Velocity true_vel = Velocity(0_mps);
  std::mt19937 generator = std::mt19937(std::random_device{}());
  std::normal_distribution distr = std::normal_distribution(0.0, 0.05);

  std::cout << "s_read,a_read,time,s_true,v_true,a_true,"
               "s_linenc,s_kalman,v_kalman,a_kalman,s_error\n";

  for (Duration sim_dur = Duration(0_s); sim_dur < SIM_DUR; sim_dur += STEP) {
    g_micros = sim_dur.as_us();
    Acceleration accel = Acceleration(distr(generator));

    accel += Acceleration(std::sin(sim_dur.as_us() * 1e-6 * M_PI));

    true_vel = true_vel + accel * static_cast<Time>(STEP);
    true_distance = true_distance 
      + true_vel * static_cast<Time>(STEP) + 
      0.5 * accel * static_cast<Time>(STEP) * static_cast<Time>(STEP);
    
    LinearEncoder::set_distance(true_distance, Timestamp(sim_dur.as_us()));
    Accelerometer::acceleration = accel;

    StateEstimation::update();

    Distance s_linenc = LinearEncoder::position();
    Distance s_kalman = StateEstimation::getPosition();
    Velocity v_kalman = StateEstimation::getVelocity();
    Acceleration a_kalman = StateEstimation::getAcceleration();
    float distance_error = static_cast<float>(true_distance - s_kalman);
    uint32_t c = LinearEncoder::stripe_count();
    if (!print) {
      std::cout << static_cast<float>(sim_dur.as_us()) / 1000000.0f << "," 
                << static_cast<float>(true_distance) << ","                
                << static_cast<float>(true_vel) << ","                     
                << static_cast<float>(accel) << ","
                << static_cast<float>(s_linenc) << ","                     
                << static_cast<float>(s_kalman) << ","                     
                << static_cast<float>(v_kalman) << ","                     
                << static_cast<float>(a_kalman) << ","                     
                << distance_error <<
                "\n";
      std::cout.flush();
    }
  }
}


int main() {
  sim2();
}
