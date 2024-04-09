#include "linear_encoder.h"
#include <iostream>

uint32_t g_time = 0;

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

Duration SIM_DUR = Duration::from_u32_s(1);
Duration STEP = Duration::from_u32_us(100);

Velocity ACCEL = Velocity::from_m_per_s(1);
Velocity MAX_VEL = Velocity::from_m_per_s(3);

Duration DECEL_TIME = Duration::from_u32_s(6);
Distance STRIDE = Distance::from_u32_mm(100); 

int main() {

  LinearEncoderBeginInfo beginInfo;
  beginInfo.left_pin = 0;
  beginInfo.right_pin = 1;
  beginInfo.stride = STRIDE;
  LinearEncoder::begin(beginInfo);

  Distance true_distance = Distance::from_u32_um(0);
  Velocity vel = Velocity::from_i32_um_per_s(0);

  std::cout << "time,velocity,distance,left,right,estimated_distance,estimated_"
               "velocity,stripe_count,isr_called,ewma_distance\n";

  uint32_t isr_called = 0;

  Distance ewma_d = Distance::from_u32_um(0);
  for (uint32_t time_us = 0; time_us < SIM_DUR.as_u32_us();
       time_us += STEP.as_u32_us()) {
    g_time = time_us;
    Velocity accel = ACCEL * STEP.as_s();

    if (time_us > DECEL_TIME.as_u32_us()) {
      if (vel.as_i32_um_per_s() > 0) {
        vel -= accel;
      }
    } else {
      if (vel.as_i32_um_per_s() < MAX_VEL.as_i32_um_per_s()) {
        vel += accel;
      }
    }
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
              << ewma_d.as_m() << "\n";
  }
  std::cout.flush();
}
