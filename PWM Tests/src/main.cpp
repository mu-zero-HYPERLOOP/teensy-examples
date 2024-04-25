#include <Arduino.h>
#include "pwm.hpp"


#define PIN_SDC 32
#define PIN_PRECHARGE_START 23
#define PIN_PRECHARGE_DONE 22
#define sqrt3over2 0.866025


pwm_control_t my_control;

float my_pwm_freq = 20000;

int counter = 0;

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

  set_PWM_freq(my_pwm_freq);
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

  digitalWrite(LED_BUILTIN, HIGH);

  delay(2000);

  Serial.println("Setup done");

  digitalWrite(PIN_SDC, HIGH);
  delay(2000);
  digitalWrite(PIN_PRECHARGE_START, HIGH);
  Serial.println("Precharge started");
  delay(3000); // duration of precharge
  digitalWrite(PIN_PRECHARGE_DONE, HIGH);
  Serial.println("Precharge done");
  delay(2000);
  Serial.println("Starting loop now");

}

void loop() {
  if(counter > 1000) {
    counter = 0;
  }
  //loop through PWM duty cycle on W2
  // my_control.U2_duty += 0.01;
  // if(my_control.U2_duty > 1) {
  //   my_control.U2_duty = 0;
  //   set_PWM_freq(my_pwm_freq);
    
  // }
  
  float theta = 3.141592 * 2.0 / 1000.0 * counter;
  float amplitude = 0.4;

  int micros_before = micros();
  // complex pointer u_a + j*u_b
  float u_a = amplitude * cos(theta) * sqrt3over2 / 2;
  float u_b = amplitude * sin(theta) * sqrt3over2 / 2;

  float u_u = u_a;
  float u_v = -0.5 * u_a + sqrt3over2 * u_b;
  float u_w = -0.5 * u_a - sqrt3over2 * u_b;

  my_control.U2_duty = 0.5 + u_u/1.0; 
  my_control.V2_duty = 0.5 + u_v/1.0; 
  my_control.W2_duty = 0.5 + u_w/1.0; 
  
  pwm_set_control(my_control);

  
  int micros_after = micros();

  Serial.printf("Theta: %f  -  U2 duty: %f  -  duration: %f\n", theta, my_control.U2_duty, micros_after - micros_before);
  delayMicroseconds(500);
  counter++;
}
