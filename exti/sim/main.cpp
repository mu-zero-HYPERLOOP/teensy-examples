#include "linear_encoder.h"
#include "metrics.h"
#include "timestamp.h"
#include <cmath>
#include <iostream>
#include "accelerometer.hpp"

uint32_t g_time = 0;

void pinMode(int, int)  {}

uint32_t micros() { return g_time; }

typedef void (*exti_isr_t)();

exti_isr_t g_exti_isr;

void attachInterrupt(int, void (*isr)(), int) { g_exti_isr = isr; }

uint8_t g_left_pin = 0;
uint8_t g_right_pin = 0;

uint8_t digitalReadFast(int pin) {
  if (pin == 0) {
    return g_left_pin;
  } else {
    return g_right_pin;
  }
}

Duration SIM_DUR = Duration(10_s);
Duration STEP = Duration(100_us);

Acceleration ACCEL = Acceleration(1_mps2);
Velocity MAX_VEL = Velocity(3_mps);

Duration DECEL_TIME = Duration::from_ms(6000);
Distance STRIDE = Distance(50_mm); 


void sim1() {

  LinearEncoder::stride = STRIDE;

  Distance true_distance = Distance(0_m);
  Velocity true_vel = Velocity(0_mps);

  std::cout << "time,velocity,distance,left,right,estimated_distance,estimated_"
               "velocity,stripe_count,isr_called,ewma_distance,distance_error\n";

  uint32_t isr_called = 0;

  Distance ewma_d = Distance(0_m);
  for (uint32_t time_us = 0; time_us < SIM_DUR.as_us();
       time_us += STEP.as_us()) {
    g_time = time_us;
    Acceleration accel = Acceleration(0_mps2);

    if (time_us > DECEL_TIME.as_us()) {
      if (true_vel > 0_mps) {
        accel = accel - 1_mps2 * static_cast<Time>(STEP) / 1_s;
      }
    } else {
      if (true_vel < MAX_VEL) {
        accel = accel + 1_mps2 * static_cast<Time>(STEP) / 1_s;
      }
    }
    true_vel = accel * static_cast<Time>(STEP);
    true_distance = true_distance + 0.5 * accel * static_cast<Time>(STEP) * static_cast<Time>(STEP);
    
    LinearEncoder::distance = true_distance;
    Accelerometer::acceleration = accel;

    Duration(1_us);

    uint32_t stride_um = STRIDE.as_u32_um();
    uint32_t period_um = stride_um;
    uint32_t td_um = true_distance.as_u32_um();

    uint32_t left_phase = td_um % (stride_um * 2);
    uint8_t left = left_phase > stride_um;

    uint32_t right_phase = (td_um + stride_um / 2) % (stride_um * 2);
    g_right_pin = right_phase > stride_um;

    if (!g_left_pin && left) { // rising edge
      g_left_pin = 1;
      isr_called += 1;
      g_exti_isr();
    } else if (g_left_pin && !left) { // falling edge
      g_left_pin = 0;
      isr_called += 1;
      g_exti_isr();
    }

    Distance d = LinearEncoder::distance();
    ewma_d = ewma_d * (0.995f) + d * (0.005f);
    float distance_error = true_distance.as_m() - d.as_m();;
    Velocity v = LinearEncoder::velocity();
    uint32_t c = LinearEncoder::stripe_count();
    std::cout << Duration::from_u32_us(time_us).as_s() << "," //
              << vel.as_m_per_s() << ","                      //
              << true_distance.as_m() << ","                  //
              << (uint32_t)left << ","                        //
              << (uint32_t)g_right_pin << ","                 //
              << d.as_m() << ","                              //
              << v.as_m_per_s() << ","                        //
              << c << ","                                     //
              << isr_called << ","                            //
              << ewma_d.as_m() << ","  //
              << distance_error <<
              "\n";
  }
  std::cout.flush();
}


void sim2() {

  LinearEncoderBeginInfo beginInfo;
  beginInfo.left_pin = 0;
  beginInfo.right_pin = 1;
  beginInfo.stride = STRIDE;
  LinearEncoder::begin(beginInfo);

  Distance true_distance = Distance::from_u32_um(0);
  Velocity vel = Velocity::from_i32_um_per_s(0);

  std::cout << "time,velocity,distance,left,right,estimated_distance,estimated_"
               "velocity,stripe_count,isr_called,ewma_distance,distance_error\n";

  uint32_t isr_called = 0;

  Distance ewma_d = Distance::from_u32_um(0);
  for (uint32_t time_us = 0; time_us < SIM_DUR.as_u32_us();
       time_us += STEP.as_u32_us()) {
    g_time = time_us;
    Velocity accel = ACCEL * (STEP.as_s() * ((float)std::cos(time_us / (M_PI * 1e5) )));

    vel += accel;
    Duration::from_u32_us(1);
    true_distance = true_distance + vel * STEP.as_s();

    uint32_t stride_um = STRIDE.as_u32_um();
    uint32_t period_um = stride_um;
    uint32_t td_um = true_distance.as_u32_um();

    uint32_t left_phase = td_um % (stride_um * 2);
    uint8_t left = left_phase > stride_um;

    uint32_t right_phase = (td_um + stride_um / 2) % (stride_um * 2);
    g_right_pin = right_phase > stride_um;

    if (!g_left_pin && left) { // rising edge
      g_left_pin = 1;
      isr_called += 1;
      g_exti_isr();
    } else if (g_left_pin && !left) { // falling edge
      g_left_pin = 0;
      isr_called += 1;
      g_exti_isr();
    }

    Distance d = LinearEncoder::distance();
    ewma_d = ewma_d * (0.995f) + d * (0.005f);
    float distance_error = true_distance.as_m() - d.as_m();;
    Velocity v = LinearEncoder::velocity();
    uint32_t c = LinearEncoder::stripe_count();
    std::cout << Duration::from_u32_us(time_us).as_s() << "," //
              << vel.as_m_per_s() << ","                      //
              << true_distance.as_m() << ","                  //
              << (uint32_t)left << ","                        //
              << (uint32_t)g_right_pin << ","                 //
              << d.as_m() << ","                              //
              << v.as_m_per_s() << ","                        //
              << c << ","                                     //
              << isr_called << ","                            //
              << ewma_d.as_m() << ","  //
              << distance_error <<
              "\n";
  }
  std::cout.flush();
}


int main() {

  sim2();
}
