#include "imxrt.h"
#include "pins_arduino.h"
#include <Arduino.h>


uint16_t PWM_SM0_Div = 4; // refer to page 3147 of reference manual
uint16_t PWM_range = 500; // PWM range from 0 to 500
uint16_t PWM_pulsewidth = 10;

void flexpwm_init_edgealigned() {     //set PWM
  FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_CLDOK(0x0F);
        // Clear Load Okay for all submodules -> no reload of PWM settings
  FLEXPWM2_SM0CTRL = FLEXPWM_SMCTRL_FULL | FLEXPWM_SMCTRL_PRSC(9); 
        // Full cycle update (@val1) and prescaler value
  FLEXPWM2_SM0CTRL2 = FLEXPWM_SMCTRL2_INDEP | FLEXPWM_SMCTRL2_CLK_SEL(0); 
        // A & B independant and use IPBus clock for counter/prescaler
  FLEXPWM2_SM0INIT = 0 ; // value to start at after reset
                         
  FLEXPWM2_SM0VAL0 = 0; // mid point for symetrical pulse around 0x00 not used
  FLEXPWM2_SM0VAL1 = 500; // value at which counter resets
  // PWM A edges
  FLEXPWM2_SM0VAL2 = FLEXPWM2_SM0INIT; // start of pulse on A
  FLEXPWM2_SM0VAL3 = FLEXPWM2_SM0INIT; // end of pulse on A
  // PWM B edges
  FLEXPWM2_SM0VAL4 = 0; // adc trigger
  FLEXPWM2_SM0VAL5 = 400; // adc trigger

  FLEXPWM2_OUTEN |= FLEXPWM_OUTEN_PWMA_EN(0x0F); // Activate all A channels
  FLEXPWM2_OUTEN |= FLEXPWM_OUTEN_PWMB_EN(0x0F); // Activate all B channels
  FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_LDOK(0x0F);
        // Give Load Okay for all submodules -> reload setting again
  FLEXPWM2_SM0TCTRL = FLEXPWM_SMTCTRL_OUT_TRIG_EN((1 << 4) | (1 << 5)); 
        // val 4 of Flexpwm sm0 as trigger for PWM_OUT_TRIG0
        // val 5 of Flexpwm sm0 as trigger for PWM_OUT_TRIG1
}

void flexpwm_init_signed() {     //set PWM
  FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_CLDOK(0x0F);
        // Clear Load Okay for all submodules -> no reload of PWM settings
  FLEXPWM2_SM1CTRL = FLEXPWM_SMCTRL_FULL | FLEXPWM_SMCTRL_PRSC(9); 
        // Full cycle update (@val1) and prescaler value
  FLEXPWM2_SM1CTRL2 = FLEXPWM_SMCTRL2_INDEP | FLEXPWM_SMCTRL2_CLK_SEL(0); 
        // A & B independant and use IPBus clock for counter/prescaler
  FLEXPWM2_SM1INIT = -128; // value to start at after reset
                         
  FLEXPWM2_SM1VAL0 = 0; // mid point for symetrical pulse around 0x00 not used
  FLEXPWM2_SM1VAL1 = -FLEXPWM2_SM1INIT; // value at which counter resets
  // PWM A edges
  FLEXPWM2_SM1VAL2 = FLEXPWM2_SM1INIT; // start of pulse on A
  FLEXPWM2_SM1VAL3 = FLEXPWM2_SM1VAL1; // end of pulse on A
  // PWM B edges
  FLEXPWM2_SM1VAL4 = -128; // adc trigger
  FLEXPWM2_SM1VAL5 = 128; // adc trigger

  FLEXPWM2_OUTEN |= FLEXPWM_OUTEN_PWMA_EN(0x0F); // Activate all A channels
  FLEXPWM2_OUTEN |= FLEXPWM_OUTEN_PWMB_EN(0x0F); // Activate all B channels
  FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_LDOK(0x0F);
        // Give Load Okay for all submodules -> reload setting again
  FLEXPWM2_SM1TCTRL = FLEXPWM_SMTCTRL_OUT_TRIG_EN((1 << 4) | (1 << 5)); 
        // val 4 of Flexpwm sm1 as trigger for PWM_OUT_TRIG0
        // val 5 of Flexpwm sm1 as trigger for PWM_OUT_TRIG1
}

void setup() {
  flexpwm_init_edgealigned();
  flexpwm_init_signed();
  *(portConfigRegister(4)) = 1; // set pin 4 as output
  *(portConfigRegister(5)) = 1; // set pin 5 as output

}


void loop() {
  FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_CLDOK(0x0f);

  FLEXPWM2_SM0VAL3++;
  if (FLEXPWM2_SM0VAL3 > PWM_range - 1) FLEXPWM2_SM0VAL3 = 0;

  FLEXPWM2_SM1VAL2++;
  if (FLEXPWM2_SM1VAL2 == 0) FLEXPWM2_SM1VAL2 = FLEXPWM2_SM1INIT;
  FLEXPWM2_SM1VAL3 = -FLEXPWM2_SM1VAL2;

  FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_LDOK(0x0f);
  delay(10);


}
