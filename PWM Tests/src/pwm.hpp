#pragma once


#include <cstdint>

struct pwm_control_t {
  float U1_duty; // range [0,1] 
  float V1_duty;
  float W1_duty;
  float U2_duty;
  float V2_duty;
  float W2_duty;
};

void pwm_init();

void pwm_enable_reload_interrupt_isr();

void pwm_disable_reload_interrupt_isr();

void set_PWM_freq(float pwm_freq);

void pwm_set_control(pwm_control_t& pwm_control);

void pwm_on_isr();
