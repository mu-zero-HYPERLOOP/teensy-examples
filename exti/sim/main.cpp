#include "linear_encoder.h"
#include "metrics.h"
#include "timestamp.h"
#include <cmath>
#include <cstdio>
#include <iostream>
#include "accelerometer.hpp"
#include "state_estimation.h"

Duration SIM_DUR = Duration(5_s);
Duration STEP = Duration(1_ms);
Acceleration ACCEL = Acceleration(1_mps2);
Velocity MAX_VEL = Velocity(3_mps);
Duration DECEL_TIME = Duration(2_s);
Distance STRIDE = Distance(5_cm); 
uint32_t g_micros = 0;

uint32_t micros() {
   return g_micros;
}


void sim1() {
  LinearEncoder::stride = STRIDE;
  StateEstimation::begin();

  Distance true_distance = Distance(0_m);
  Velocity true_vel = Velocity(0_mps);

  std::cout << "time,true_distance,true_velocity,true_acceleration,"
               "d_linenc,d_kalman,d_error\n";

  for (Duration sim_dur = Duration(0_s); sim_dur < SIM_DUR; sim_dur += STEP) {
    g_micros = sim_dur.as_us();
    Acceleration accel = Acceleration(0_mps2);

    if (sim_dur > DECEL_TIME) {
      if (true_vel > 0_mps) {
        accel = accel - 1_mps2;
      }
    } else {
      if (true_vel < MAX_VEL) {
        accel = accel + 1_mps2;
      }
    }
    true_vel = true_vel + accel * static_cast<Time>(STEP);
    true_distance = true_distance 
      + true_vel * static_cast<Time>(STEP) + 
      0.5 * accel * static_cast<Time>(STEP) * static_cast<Time>(STEP);
    //std::printf("true distance: %f\n", static_cast<float>(true_distance));
    //std::printf("stripes coutned: %d\n", LinearEncoder::stripe_count());
    
    LinearEncoder::set_distance(true_distance, Timestamp(sim_dur.as_us()));
    Accelerometer::acceleration = accel;

    StateEstimation::update();

    Distance d_linenc = LinearEncoder::position();
    Distance d_kalman = StateEstimation::getPosition();
    float distance_error = static_cast<float>(true_distance - d_kalman);
    uint32_t c = LinearEncoder::stripe_count();
    std::cout << static_cast<float>(sim_dur.as_us()) / 1000000.0f << "," 
              << static_cast<float>(true_distance) << ","                
              << static_cast<float>(true_vel) << ","                     
              << static_cast<float>(accel) << ","
              << static_cast<float>(d_linenc) << ","                     
              << static_cast<float>(d_kalman) << ","                     
              << distance_error <<
              "\n";
  }
  std::cout.flush();
}




int main() {
  sim1();
}
