#include "pwm.hpp"

#include "xbar.hpp"

#include <Arduino.h>
#include <imxrt.h>

volatile float PWM_FREQUENCY = 20000.0;
static constexpr uint16_t PWM_DEADTIME_NS = 1000; // nanoseconds

volatile int PWMCycles = (uint32_t)((float)F_BUS_ACTUAL / (2 * PWM_FREQUENCY) + 0.5);

void pwm_init() {

  // this function should be independent of the assignment of the phases (U1, V1, W1, U2, V2, W2) to the submodules
  // the only place this assignment should happen is in the pwm_set_control function

  // make xbar connections for synchronization of modules
  xbar_init();

  int Prescaler = 0;

  // Falls Frequenz zu gering Prescaler switchen!
  while (PWMCycles > 32767 && Prescaler < 7) {
    PWMCycles = PWMCycles / 2;
    Prescaler = Prescaler + 1;
  }
  if (PWMCycles > 32767) {
    PWMCycles = 32767;
  } else if (PWMCycles < 2) {
    PWMCycles = 2; // minimale Cycles --> 10nS oder so
  }

  // default values 
  int Cycles_PWM2_SM0 = PWMCycles / 2;
  int Cycles_PWM2_SM2 = PWMCycles / 2;
  int Cycles_PWM2_SM3 = PWMCycles / 2;
  int Cycles_PWM4_SM2 = PWMCycles / 2;
  int Cycles_PWM3_SM1 = PWMCycles / 2;
  int Cycles_PWM1_SM3 = PWMCycles / 2;

  int CyclesDEADTIME =
      (int)((float)F_BUS_ACTUAL * PWM_DEADTIME_NS * 0.000000001);

  // initalize Flex PWM modules
  // see https://www.pjrc.com/teensy/IMXRT1060RM_rev3.pdf at p3135 for register
  // description

  // NOTE i formatted this a little bit to hopefully make more sense
  // at the start i just initalize all PWM modules so : clock source, prescalar,
  // deadtime, INIT, VAL1 (max_counter), VAL0. Afterwards we can to the actual
  // configuring of the individual modules.

  // Clear Load Okay -> no reload of PWM settings.
  FLEXPWM1_MCTRL |= FLEXPWM_MCTRL_CLDOK(0x0F);
  FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_CLDOK(0x0F);
  FLEXPWM3_MCTRL |= FLEXPWM_MCTRL_CLDOK(0x0F);
  FLEXPWM4_MCTRL |= FLEXPWM_MCTRL_CLDOK(0x0F);

  // SMxCTRL:
  //  - FULL : Full Cycle Reload enabled.
  //  - PRSC : Prescalar (3bit) : PWM Clock Frequency is f = f_clk / (2 <<
  //  Prescalar - 1).

  FLEXPWM1_SM3CTRL = FLEXPWM_SMCTRL_FULL | FLEXPWM_SMCTRL_PRSC(Prescaler);
  FLEXPWM2_SM0CTRL = FLEXPWM_SMCTRL_FULL | FLEXPWM_SMCTRL_PRSC(Prescaler);
  FLEXPWM2_SM2CTRL = FLEXPWM_SMCTRL_FULL | FLEXPWM_SMCTRL_PRSC(Prescaler);
  FLEXPWM2_SM3CTRL = FLEXPWM_SMCTRL_FULL | FLEXPWM_SMCTRL_PRSC(Prescaler);
  FLEXPWM3_SM1CTRL = FLEXPWM_SMCTRL_FULL | FLEXPWM_SMCTRL_PRSC(Prescaler);
  FLEXPWM4_SM2CTRL = FLEXPWM_SMCTRL_FULL | FLEXPWM_SMCTRL_PRSC(Prescaler);

  
  FLEXPWM4_SM0CTRL = FLEXPWM_SMCTRL_FULL | FLEXPWM_SMCTRL_PRSC(Prescaler);


  // CTRL2:
  //  -CLK_SEC : Selects the clock source. 0 selects the IPBus clock source.
  //  -INIT_SEL : Initalization Control Select :
  //        0b00 selects Local sync (PWM_X) cases initalization
  //        0b10 selects Master sync from SM0 causes initalization.
  //        0b11 selects EXT_SYC causes initalization.
  //  -RELOAD_SEL : The master RELOAD from SM0 is used.
  // Why do different submodules have different settings here??
  // -> different sources for syncronization
  // PWM4_SM2 is the source for sync

  // routing the EXT_SYNC over XBAR introduces a small delay, of ~28ns
  // different to SciMo, now every submodule is reset by EXT_SYNC, which originates (for all SM) from PWM4_SM0 so every submodule is triggered at the same time
  FLEXPWM1_SM3CTRL2 = FLEXPWM_SMCTRL2_CLK_SEL(0) | FLEXPWM_SMCTRL2_INIT_SEL(0b11);
  FLEXPWM2_SM0CTRL2 = FLEXPWM_SMCTRL2_CLK_SEL(0) | FLEXPWM_SMCTRL2_INIT_SEL(0b11);
  FLEXPWM2_SM2CTRL2 = FLEXPWM_SMCTRL2_CLK_SEL(0) | FLEXPWM_SMCTRL2_INIT_SEL(0b11);
  FLEXPWM2_SM3CTRL2 = FLEXPWM_SMCTRL2_CLK_SEL(0) | FLEXPWM_SMCTRL2_INIT_SEL(0b11);
  FLEXPWM3_SM1CTRL2 = FLEXPWM_SMCTRL2_CLK_SEL(0) | FLEXPWM_SMCTRL2_INIT_SEL(0b11);
  FLEXPWM4_SM2CTRL2 = FLEXPWM_SMCTRL2_CLK_SEL(0) | FLEXPWM_SMCTRL2_INIT_SEL(0b11);

  FLEXPWM4_SM0CTRL2 = FLEXPWM_SMCTRL2_CLK_SEL(0);

  // Enable outputs
  // TODO use me instead, but check before changing anything
  // "FLEXPWM_OUTEN_PWMA_EN(0b0111) | FLEXPWM_OUTEN_PWMB_EN(0b0111);"

  // jeweils ENA ENB ENX    ACHTUNG SM0 bis SM3
  FLEXPWM1_OUTEN = 0b1000'1000'0000;
  FLEXPWM2_OUTEN = 0b1101'1101'0000;
  FLEXPWM3_OUTEN = 0b0010'0010'0000;
  FLEXPWM4_OUTEN = 0b0100'0100'0000; // TODO why enable submodules 0 and 1?

  // TODO find out what this is actually used for maybe it is also used for
  // pwm synchronization, but iam not sure yet.
  // Enable triggers for submodule at val4 (TODO probably for ADC convertion)
  FLEXPWM4_SM0TCTRL = FLEXPWM_SMTCTRL_OUT_TRIG_EN(1 << 4);
  // FLEXPWM4_SM1TCTRL = FLEXPWM_SMTCTRL_OUT_TRIG_EN(1 << 4);

  // Enable reload Interrupt for PWM4_SM2
  // FLEXPWM4_SM2INTEN |= FLEXPWM_SMINTEN_CMPIE(1 << 4); // trigger at val 4
  // FLEXPWM4_SM2VAL4 = -PWMCycles;

  // SMxDTCNTx Set the Deadtime of the complementary PWM pair.
  // PWM 1 SM 3
  FLEXPWM1_SM3DTCNT0 = CyclesDEADTIME;
  FLEXPWM1_SM3DTCNT1 = CyclesDEADTIME;
  // PWM 2 SM 0
  FLEXPWM2_SM0DTCNT0 = CyclesDEADTIME;
  FLEXPWM2_SM0DTCNT1 = CyclesDEADTIME;
  // PWM 2 SM 2
  FLEXPWM2_SM2DTCNT0 = CyclesDEADTIME;
  FLEXPWM2_SM2DTCNT1 = CyclesDEADTIME;
  // PWM w SM 3
  FLEXPWM2_SM3DTCNT0 = CyclesDEADTIME;
  FLEXPWM2_SM3DTCNT1 = CyclesDEADTIME;
  // PWM 3 SM 1
  FLEXPWM3_SM1DTCNT0 = CyclesDEADTIME;
  FLEXPWM3_SM1DTCNT1 = CyclesDEADTIME;
  // PWM 4 SM 2
  FLEXPWM4_SM2DTCNT0 = CyclesDEADTIME;
  FLEXPWM4_SM2DTCNT1 = CyclesDEADTIME;

  // Initalize PWM for a symetrical pulse around 0x0.
  //  - INIT is reload value (or start value)
  //  - VAL0 is the mid point
  //  - VAL1 is the max count before counter reset.
  // PWM 1 SM 3
  FLEXPWM1_SM3INIT = -PWMCycles;
  FLEXPWM1_SM3VAL0 = 0;
  FLEXPWM1_SM3VAL1 = PWMCycles;
  // PWM 2 SM 0
  FLEXPWM2_SM0INIT = -PWMCycles;
  FLEXPWM2_SM0VAL0 = 0;
  FLEXPWM2_SM0VAL1 = PWMCycles;
  // PWM 2 SM 2
  FLEXPWM2_SM2INIT = -PWMCycles;
  FLEXPWM2_SM2VAL0 = 0;
  FLEXPWM2_SM2VAL1 = PWMCycles;
  // PWM 2 SM 3
  FLEXPWM2_SM3INIT = -PWMCycles;
  FLEXPWM2_SM3VAL0 = 0;
  FLEXPWM2_SM3VAL1 = PWMCycles;
  // PWM 3 SM 1
  FLEXPWM3_SM1INIT = -PWMCycles;
  FLEXPWM3_SM1VAL0 = 0;
  FLEXPWM3_SM1VAL1 = PWMCycles;
  // PWM 4 SM 2
  FLEXPWM4_SM2INIT = -PWMCycles;
  FLEXPWM4_SM2VAL0 = 0;
  FLEXPWM4_SM2VAL1 = PWMCycles;

  // PWM 4 SM 0
  // TODO what is this submodule used for exactly.
  FLEXPWM4_SM0INIT = -PWMCycles;
  FLEXPWM4_SM0VAL1 = PWMCycles;
  // PWM 4 SM 1
  // TODO same here!
  FLEXPWM4_SM1INIT = -PWMCycles;
  FLEXPWM4_SM1VAL1 = PWMCycles;



  // PWM 4 SM 2
  FLEXPWM4_SM2VAL2 = -Cycles_PWM4_SM2;
  FLEXPWM4_SM2VAL3 = Cycles_PWM4_SM2;

  FLEXPWM4_SM0VAL4 = -PWMCycles;

  FLEXPWM4_SM1VAL4 = -PWMCycles; // Interliev AUS

  FLEXPWM3_SM1VAL2 = -Cycles_PWM3_SM1;
  FLEXPWM3_SM1VAL3 = Cycles_PWM3_SM1;
  FLEXPWM3_SM1VAL4 = 0;
  FLEXPWM3_SM1VAL5 = 0;

  FLEXPWM1_SM3VAL2 = -Cycles_PWM1_SM3;
  FLEXPWM1_SM3VAL3 = Cycles_PWM1_SM3;
  FLEXPWM1_SM3VAL4 = 0; // adc trigger
  FLEXPWM1_SM3VAL5 = 0; // adc trigger

  FLEXPWM2_SM0VAL2 = -Cycles_PWM2_SM0;
  FLEXPWM2_SM0VAL3 = Cycles_PWM2_SM0;
  FLEXPWM2_SM0VAL4 = 0; // adc trigger
  FLEXPWM2_SM0VAL5 = 0; // adc trigger

  FLEXPWM2_SM2VAL2 = -Cycles_PWM2_SM2;
  FLEXPWM2_SM2VAL3 = Cycles_PWM2_SM2;
  FLEXPWM2_SM2VAL4 = 0; // adc trigger
  FLEXPWM2_SM2VAL5 = 0; // adc trigger

  FLEXPWM2_SM3VAL2 = -Cycles_PWM2_SM3;
  FLEXPWM2_SM3VAL3 = Cycles_PWM2_SM3;
  FLEXPWM2_SM3VAL4 = 0; // adc trigger
  FLEXPWM2_SM3VAL5 = 0; // adc trigger

  /////// Phase U2   PWM4 SM 2////////////////
  // NOTE: used to trigger compare PWM_OUT_TRIG0

  /////// Phase V2   PWM3 SM 1////////////////
  // NOTE used to trigger TODO what the fuck is this adc trigger stuff don't get
  // it.

  /////// Phase W2   PWM1 SM 3////////////////

  /////// Phase U1   PWM2 SM 0////////////////
  // TODO super interessting =^). How the ADC works is still unclear to me.

  /////// Phase V1   PWM2 SM 2////////////////

  /////// Phase W1   PWM2 SM 3////////////////

  // Update pin configuration
  // Selects which module is connected to the pins, in this case it is the flexpwm module
  // the assigned values are from the datasheet chapter 11.6.xx and dont follow an obvious rule i think
  *(portConfigRegister(2)) = 1;
  *(portConfigRegister(3)) = 1;

  *(portConfigRegister(28)) = 1;
  *(portConfigRegister(29)) = 1;

  *(portConfigRegister(7)) = 6;
  *(portConfigRegister(8)) = 6;

  *(portConfigRegister(4)) = 1;
  *(portConfigRegister(33)) = 1;

  *(portConfigRegister(6)) = 2;
  *(portConfigRegister(9)) = 2;

  *(portConfigRegister(36)) = 6;
  *(portConfigRegister(37)) = 6;

  // Load Okay -> reload setting again
  FLEXPWM4_MCTRL |= FLEXPWM_MCTRL_LDOK(0x0F);
  FLEXPWM3_MCTRL |= FLEXPWM_MCTRL_LDOK(0x0F);
  FLEXPWM1_MCTRL |= FLEXPWM_MCTRL_LDOK(0x0F);
  FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_LDOK(0x0F);
}

void set_PWM_freq(float pwm_freq) {

  PWM_FREQUENCY = pwm_freq;

  PWMCycles = (uint32_t)((float)F_BUS_ACTUAL / (2 * PWM_FREQUENCY) + 0.5);

  int Prescaler = 0;

  // Falls Frequenz zu gering Prescaler switchen!
  while (PWMCycles > 32767 && Prescaler < 7) {
    PWMCycles = PWMCycles / 2;
    Prescaler = Prescaler + 1;
  }
  if (PWMCycles > 32767) {
    PWMCycles = 32767;
  } else if (PWMCycles < 2) {
    PWMCycles = 2; // minimale Cycles --> 10nS oder so
  }

  // default values 
  int Cycles_PWM2_SM0 = PWMCycles / 2;
  int Cycles_PWM2_SM2 = PWMCycles / 2;
  int Cycles_PWM2_SM3 = PWMCycles / 2;
  int Cycles_PWM4_SM2 = PWMCycles / 2;
  int Cycles_PWM3_SM1 = PWMCycles / 2;
  int Cycles_PWM1_SM3 = PWMCycles / 2;

  int CyclesDEADTIME =
      (int)((float)F_BUS_ACTUAL * PWM_DEADTIME_NS * 0.000000001);

  // initalize Flex PWM modules
  // see https://www.pjrc.com/teensy/IMXRT1060RM_rev3.pdf at p3135 for register
  // description

  // NOTE i formatted this a little bit to hopefully make more sense
  // at the start i just initalize all PWM modules so : clock source, prescalar,
  // deadtime, INIT, VAL1 (max_counter), VAL0. Afterwards we can to the actual
  // configuring of the individual modules.

  // Clear Load Okay -> no reload of PWM settings.
  FLEXPWM1_MCTRL |= FLEXPWM_MCTRL_CLDOK(0x0F);
  FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_CLDOK(0x0F);
  FLEXPWM3_MCTRL |= FLEXPWM_MCTRL_CLDOK(0x0F);
  FLEXPWM4_MCTRL |= FLEXPWM_MCTRL_CLDOK(0x0F);

  // SMxCTRL:
  //  - FULL : Full Cycle Reload enabled.
  //  - HALF : Reload at half cycle -> this is necessary to be able to increase the PWM frequency
  //    (with full cycle reload it does not work, probably because of the way the synchronization works)
  //  - PRSC : Prescalar (3bit) : PWM Clock Frequency is f = f_clk / (2 <<
  //  Prescalar - 1).

  FLEXPWM1_SM3CTRL = FLEXPWM_SMCTRL_HALF | FLEXPWM_SMCTRL_PRSC(Prescaler);
  FLEXPWM2_SM0CTRL = FLEXPWM_SMCTRL_HALF | FLEXPWM_SMCTRL_PRSC(Prescaler);
  FLEXPWM2_SM2CTRL = FLEXPWM_SMCTRL_HALF | FLEXPWM_SMCTRL_PRSC(Prescaler);
  FLEXPWM2_SM3CTRL = FLEXPWM_SMCTRL_HALF | FLEXPWM_SMCTRL_PRSC(Prescaler);
  FLEXPWM3_SM1CTRL = FLEXPWM_SMCTRL_HALF | FLEXPWM_SMCTRL_PRSC(Prescaler);
  FLEXPWM4_SM2CTRL = FLEXPWM_SMCTRL_HALF | FLEXPWM_SMCTRL_PRSC(Prescaler);

  
  FLEXPWM4_SM0CTRL = FLEXPWM_SMCTRL_HALF | FLEXPWM_SMCTRL_PRSC(Prescaler);


  // CTRL2:
  //  -CLK_SEC : Selects the clock source. 0 selects the IPBus clock source.
  //  -INIT_SEL : Initalization Control Select :
  //        0b00 selects Local sync (PWM_X) cases initalization
  //        0b10 selects Master sync from SM0 causes initalization.
  //        0b11 selects EXT_SYC causes initalization.
  //  -RELOAD_SEL : The master RELOAD from SM0 is used.
  // Why do different submodules have different settings here??
  // -> different sources for syncronization
  // PWM4_SM2 is the source for sync

  // routing the EXT_SYNC over XBAR introduces a small delay, of ~28ns
  // different to SciMo, now every submodule is reset by EXT_SYNC, which originates (for all SM) from PWM4_SM0 so every submodule is triggered at the same time
  FLEXPWM1_SM3CTRL2 = FLEXPWM_SMCTRL2_CLK_SEL(0) | FLEXPWM_SMCTRL2_INIT_SEL(0b11);
  FLEXPWM2_SM0CTRL2 = FLEXPWM_SMCTRL2_CLK_SEL(0) | FLEXPWM_SMCTRL2_INIT_SEL(0b11);
  FLEXPWM2_SM2CTRL2 = FLEXPWM_SMCTRL2_CLK_SEL(0) | FLEXPWM_SMCTRL2_INIT_SEL(0b11);
  FLEXPWM2_SM3CTRL2 = FLEXPWM_SMCTRL2_CLK_SEL(0) | FLEXPWM_SMCTRL2_INIT_SEL(0b11);
  FLEXPWM3_SM1CTRL2 = FLEXPWM_SMCTRL2_CLK_SEL(0) | FLEXPWM_SMCTRL2_INIT_SEL(0b11);
  FLEXPWM4_SM2CTRL2 = FLEXPWM_SMCTRL2_CLK_SEL(0) | FLEXPWM_SMCTRL2_INIT_SEL(0b11);

  FLEXPWM4_SM0CTRL2 = FLEXPWM_SMCTRL2_CLK_SEL(0);

  // Enable outputs
  // TODO use me instead, but check before changing anything
  // "FLEXPWM_OUTEN_PWMA_EN(0b0111) | FLEXPWM_OUTEN_PWMB_EN(0b0111);"

  // jeweils ENA ENB ENX    ACHTUNG SM0 bis SM3
  FLEXPWM1_OUTEN = 0b1000'1000'0000;
  FLEXPWM2_OUTEN = 0b1101'1101'0000;
  FLEXPWM3_OUTEN = 0b0010'0010'0000;
  FLEXPWM4_OUTEN = 0b0100'0100'0000; // TODO why enable submodules 0 and 1?

  // TODO find out what this is actually used for maybe it is also used for
  // pwm synchronization, but iam not sure yet.
  // Enable triggers for submodule at val4 (TODO probably for ADC convertion)
  FLEXPWM4_SM0TCTRL = FLEXPWM_SMTCTRL_OUT_TRIG_EN(1 << 4);
  // FLEXPWM4_SM1TCTRL = FLEXPWM_SMTCTRL_OUT_TRIG_EN(1 << 4);

  // Enable reload Interrupt for PWM4_SM2
  // FLEXPWM4_SM2INTEN |= FLEXPWM_SMINTEN_CMPIE(1 << 4); // trigger at val 4
  // FLEXPWM4_SM2VAL4 = -PWMCycles;

  // SMxDTCNTx Set the Deadtime of the complementary PWM pair.
  // PWM 1 SM 3
  FLEXPWM1_SM3DTCNT0 = CyclesDEADTIME;
  FLEXPWM1_SM3DTCNT1 = CyclesDEADTIME;
  // PWM 2 SM 0
  FLEXPWM2_SM0DTCNT0 = CyclesDEADTIME;
  FLEXPWM2_SM0DTCNT1 = CyclesDEADTIME;
  // PWM 2 SM 2
  FLEXPWM2_SM2DTCNT0 = CyclesDEADTIME;
  FLEXPWM2_SM2DTCNT1 = CyclesDEADTIME;
  // PWM w SM 3
  FLEXPWM2_SM3DTCNT0 = CyclesDEADTIME;
  FLEXPWM2_SM3DTCNT1 = CyclesDEADTIME;
  // PWM 3 SM 1
  FLEXPWM3_SM1DTCNT0 = CyclesDEADTIME;
  FLEXPWM3_SM1DTCNT1 = CyclesDEADTIME;
  // PWM 4 SM 2
  FLEXPWM4_SM2DTCNT0 = CyclesDEADTIME;
  FLEXPWM4_SM2DTCNT1 = CyclesDEADTIME;

  // Initalize PWM for a symetrical pulse around 0x0.
  //  - INIT is reload value (or start value)
  //  - VAL0 is the mid point
  //  - VAL1 is the max count before counter reset.
  // PWM 1 SM 3
  FLEXPWM1_SM3INIT = -PWMCycles;
  FLEXPWM1_SM3VAL0 = 0;
  FLEXPWM1_SM3VAL1 = PWMCycles;
  // PWM 2 SM 0
  FLEXPWM2_SM0INIT = -PWMCycles;
  FLEXPWM2_SM0VAL0 = 0;
  FLEXPWM2_SM0VAL1 = PWMCycles;
  // PWM 2 SM 2
  FLEXPWM2_SM2INIT = -PWMCycles;
  FLEXPWM2_SM2VAL0 = 0;
  FLEXPWM2_SM2VAL1 = PWMCycles;
  // PWM 2 SM 3
  FLEXPWM2_SM3INIT = -PWMCycles;
  FLEXPWM2_SM3VAL0 = 0;
  FLEXPWM2_SM3VAL1 = PWMCycles;
  // PWM 3 SM 1
  FLEXPWM3_SM1INIT = -PWMCycles;
  FLEXPWM3_SM1VAL0 = 0;
  FLEXPWM3_SM1VAL1 = PWMCycles;
  // PWM 4 SM 2
  FLEXPWM4_SM2INIT = -PWMCycles;
  FLEXPWM4_SM2VAL0 = 0;
  FLEXPWM4_SM2VAL1 = PWMCycles;

  // PWM 4 SM 0
  // TODO what is this submodule used for exactly.
  FLEXPWM4_SM0INIT = -PWMCycles;
  FLEXPWM4_SM0VAL1 = PWMCycles;
  // PWM 4 SM 1
  // TODO same here!
  FLEXPWM4_SM1INIT = -PWMCycles;
  FLEXPWM4_SM1VAL1 = PWMCycles;



  // PWM 4 SM 2
  FLEXPWM4_SM2VAL2 = -Cycles_PWM4_SM2;
  FLEXPWM4_SM2VAL3 = Cycles_PWM4_SM2;

  FLEXPWM4_SM0VAL4 = -PWMCycles;

  FLEXPWM4_SM1VAL4 = -PWMCycles; // Interliev AUS

  FLEXPWM3_SM1VAL2 = -Cycles_PWM3_SM1;
  FLEXPWM3_SM1VAL3 = Cycles_PWM3_SM1;
  FLEXPWM3_SM1VAL4 = 0;
  FLEXPWM3_SM1VAL5 = 0;

  FLEXPWM1_SM3VAL2 = -Cycles_PWM1_SM3;
  FLEXPWM1_SM3VAL3 = Cycles_PWM1_SM3;
  FLEXPWM1_SM3VAL4 = 0; // adc trigger
  FLEXPWM1_SM3VAL5 = 0; // adc trigger

  FLEXPWM2_SM0VAL2 = -Cycles_PWM2_SM0;
  FLEXPWM2_SM0VAL3 = Cycles_PWM2_SM0;
  FLEXPWM2_SM0VAL4 = 0; // adc trigger
  FLEXPWM2_SM0VAL5 = 0; // adc trigger

  FLEXPWM2_SM2VAL2 = -Cycles_PWM2_SM2;
  FLEXPWM2_SM2VAL3 = Cycles_PWM2_SM2;
  FLEXPWM2_SM2VAL4 = 0; // adc trigger
  FLEXPWM2_SM2VAL5 = 0; // adc trigger

  FLEXPWM2_SM3VAL2 = -Cycles_PWM2_SM3;
  FLEXPWM2_SM3VAL3 = Cycles_PWM2_SM3;
  FLEXPWM2_SM3VAL4 = 0; // adc trigger
  FLEXPWM2_SM3VAL5 = 0; // adc trigger

  /////// Phase U2   PWM4 SM 2////////////////
  // NOTE: used to trigger compare PWM_OUT_TRIG0

  /////// Phase V2   PWM3 SM 1////////////////
  // NOTE used to trigger TODO what the fuck is this adc trigger stuff don't get
  // it.

  /////// Phase W2   PWM1 SM 3////////////////

  /////// Phase U1   PWM2 SM 0////////////////
  // TODO super interessting =^). How the ADC works is still unclear to me.

  /////// Phase V1   PWM2 SM 2////////////////

  /////// Phase W1   PWM2 SM 3////////////////

  // Update pin configuration
  // Selects which module is connected to the pins, in this case it is the flexpwm module
  // the assigned values are from the datasheet chapter 11.6.xx and dont follow an obvious rule i think
  *(portConfigRegister(2)) = 1;
  *(portConfigRegister(3)) = 1;

  *(portConfigRegister(28)) = 1;
  *(portConfigRegister(29)) = 1;

  *(portConfigRegister(7)) = 6;
  *(portConfigRegister(8)) = 6;

  *(portConfigRegister(4)) = 1;
  *(portConfigRegister(33)) = 1;

  *(portConfigRegister(6)) = 2;
  *(portConfigRegister(9)) = 2;

  *(portConfigRegister(36)) = 6;
  *(portConfigRegister(37)) = 6;

  // Load Okay -> reload setting again
  FLEXPWM4_MCTRL |= FLEXPWM_MCTRL_LDOK(0x0F);
  FLEXPWM3_MCTRL |= FLEXPWM_MCTRL_LDOK(0x0F);
  FLEXPWM1_MCTRL |= FLEXPWM_MCTRL_LDOK(0x0F);
  FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_LDOK(0x0F);
}

void pwm_set_control(pwm_control_t &pwm_control) {
  // phases are assigned to submodules here
  // pins are mentioned in order "H , L"


  // U1
  // FlexPWM 2 Submodule 0  /  Pins 4 , 33
  uint16_t pwm2_sm0_cmp = pwm_control.U1_duty * PWMCycles;
  if (pwm2_sm0_cmp > PWMCycles){ // clamp between 0 and PWMCycles both inclusive
    pwm2_sm0_cmp = PWMCycles; 
  }
  FLEXPWM2_SM0VAL2 = -pwm2_sm0_cmp; // start uf pulse of t1.
  FLEXPWM2_SM0VAL3 = pwm2_sm0_cmp;  // end of pulse of t1.


  // V2
  // FlexPWM 2 Submodule 2  /  Pins 6 , 9
  uint16_t pwm2_sm2_cmp = pwm_control.V2_duty * PWMCycles;
  if (pwm2_sm2_cmp > PWMCycles) {
    pwm2_sm2_cmp = PWMCycles;
  }
  FLEXPWM2_SM2VAL2 = -pwm2_sm2_cmp; 
  FLEXPWM2_SM2VAL3 = pwm2_sm2_cmp; 


  // U2
  // FlexPWM 2 Submodule 3  /  Pins 36 , 37
  uint16_t pwm2_sm3_cmp = pwm_control.U2_duty * PWMCycles;
  if (pwm2_sm3_cmp > PWMCycles) {
    pwm2_sm3_cmp = PWMCycles;
  }
  FLEXPWM2_SM3VAL2 = -pwm2_sm3_cmp; 
  FLEXPWM2_SM3VAL3 = pwm2_sm3_cmp; 


  // V1
  // FlexPWM 1 Submodule 3  /  Pins 8 , 7
  uint16_t pwm1_sm3_cmp = pwm_control.V1_duty * PWMCycles;
  if (pwm1_sm3_cmp > PWMCycles) {
    pwm1_sm3_cmp = PWMCycles;
  }
  FLEXPWM1_SM3VAL2 = -pwm1_sm3_cmp; 
  FLEXPWM1_SM3VAL3 = pwm1_sm3_cmp; 


  // W1
  // FlexPWM 3 Submodule 1  /  Pins 29 , 28
  uint16_t pwm3_sm1_cmp = pwm_control.W1_duty * PWMCycles;
  if (pwm3_sm1_cmp > PWMCycles) {
    pwm3_sm1_cmp = PWMCycles;
  }
  FLEXPWM3_SM1VAL2 = -pwm3_sm1_cmp; 
  FLEXPWM3_SM1VAL3 = pwm3_sm1_cmp; 


  // W2
  // FlexPWM 4 Submodule 2  /  Pins 2 , 3
  uint16_t pwm4_sm2_val = pwm_control.W2_duty * PWMCycles;
  if (pwm4_sm2_val > PWMCycles) {
    pwm4_sm2_val = PWMCycles;
  }
  FLEXPWM4_SM2VAL2 = -pwm4_sm2_val; 
  FLEXPWM4_SM2VAL3 = pwm4_sm2_val; 

  // Load Okay -> reload setting again
  FLEXPWM4_MCTRL |= FLEXPWM_MCTRL_LDOK(0x0F);
  FLEXPWM3_MCTRL |= FLEXPWM_MCTRL_LDOK(0x0F);
  FLEXPWM1_MCTRL |= FLEXPWM_MCTRL_LDOK(0x0F);
  FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_LDOK(0x0F);
}

static void pwm4_sm2_isr() { pwm_on_isr(); }

void pwm_enable_reload_interrupt_isr() {
  FLEXPWM4_SM2INTEN |= FLEXPWM_SMINTEN_CMPIE(1 << 4); // Enable reload interrupt
  // TODO check if we can to this once at init because seems kind of suss to
  // register the isr again and again.
  attachInterruptVector(IRQ_FLEXPWM4_2, pwm4_sm2_isr);
  NVIC_ENABLE_IRQ(IRQ_FLEXPWM4_2);
}

void pwm_disable_reload_interrupt_isr() {
  FLEXPWM4_SM2INTEN |=
      FLEXPWM_SMINTEN_CMPIE(0 << 4); // Disable reload interrupt
}

void __attribute__((weak)) pwm_on_isr() {}
