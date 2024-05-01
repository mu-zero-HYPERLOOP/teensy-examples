#include <Arduino.h>
#include "pwm.hpp"


#define PIN_SDC 32
#define PIN_PRECHARGE_START 23
#define PIN_PRECHARGE_DONE 22
#define sqrt3over2 0.866025


pwm_control_t my_control;

volatile float my_pwm_freq = 20000;

volatile float mainloop_counter = 0;
volatile float theta = 0;

volatile float rotational_freq = 1;
volatile float modulation_index = 0.3;

void pwm_on_isr() {
  FLEXPWM4_SM0STS |= FLEXPWM_SMSTS_CMPF(4); // Clear interrupt flag
  // do shit here
  pwm_control_t isr_control;

  theta += rotational_freq * TWO_PI / my_pwm_freq;
  if(theta > TWO_PI) {
    theta -= TWO_PI;
  }

  // complex pointer u_a + j*u_b
  float u_a = modulation_index * cos(theta) * sqrt3over2 / 2;
  float u_b = modulation_index * sin(theta) * sqrt3over2 / 2;

  float u_u = u_a;
  float u_v = -0.5 * u_a + sqrt3over2 * u_b;
  float u_w = -0.5 * u_a - sqrt3over2 * u_b;

  isr_control.U2_duty = 0.5 + u_u/1.0; 
  isr_control.V2_duty = 0.5 + u_v/1.0; 
  isr_control.W2_duty = 0.5 + u_w/1.0; 

  digitalWrite(LED_BUILTIN, HIGH);

  pwm_set_control(isr_control);
}

void setup() {
  
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_SDC, OUTPUT);
  pinMode(PIN_PRECHARGE_START, OUTPUT);
  pinMode(PIN_PRECHARGE_DONE, OUTPUT);
  
  digitalWrite(PIN_SDC, LOW);
  digitalWrite(PIN_PRECHARGE_START, LOW);
  digitalWrite(PIN_PRECHARGE_DONE, LOW);

  delay(2000);

  pwm_init();

  // set_PWM_freq(my_pwm_freq);
  // Serial.println("PWM freq set");
  delay(10); // delay required !!! pwm reload or some shit

  my_control.U1_duty = 0; // pins 4, 33
  my_control.U2_duty = 0; // pins 36, 37
  my_control.V1_duty = 0; // pins 8 , 7
  my_control.V2_duty = 0; // pins 6 , 9
  my_control.W1_duty = 0; // pins 29, 28
  my_control.W2_duty = 0; // pins 2 , 3
  pwm_set_control(my_control);

  Serial.println("PWM control set");

  pwm_enable_reload_interrupt_isr();

  Serial.println("PWM interrupt set");

  digitalWrite(LED_BUILTIN, LOW);

  delay(2000);

  Serial.println("Setup done");

  digitalWrite(PIN_SDC, HIGH);
  delay(2000);
  digitalWrite(PIN_PRECHARGE_START, HIGH);
  Serial.println("Precharge started");
  delay(3000); // duration of precharge
  digitalWrite(PIN_PRECHARGE_DONE, HIGH);
  delay(500);
  digitalWrite(PIN_PRECHARGE_START, LOW);
  Serial.println("Precharge done");
  delay(5000);
  Serial.println("Starting loop now");
  pwm_enable_output();
}

void loop() {
  if(mainloop_counter > 100) {
    mainloop_counter = 0;
  }

  float speed = 0;
  if(mainloop_counter < 50) {
    // increase frequency
    speed = 1.5 * mainloop_counter / 50.0;
  } else {
    // decrease frequency
    speed = 1.5 * (2 - mainloop_counter / 50.0);
  }
  rotational_freq = speed * 5 + 1;
  if(speed < 0.2) {
    // lift voltage for low speeds
    modulation_index = speed * 0.25 + 0.05;

  } else {
    modulation_index = speed * 0.5;
  }



  Serial.printf("Freq: %f  -  mod index: %f \n", rotational_freq, modulation_index);
  delay(100);
  mainloop_counter += 1;

  // Serial.println("mainloop");
}

