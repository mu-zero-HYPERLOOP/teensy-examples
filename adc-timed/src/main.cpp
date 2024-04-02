#include "imxrt.h"
#include "pins_arduino.h"
#include <Arduino.h>


uint16_t PWM_SM0_Div = 4; // refer to page 3147 of reference manual
uint16_t PWM_range = 500; // PWM range from 0 to 500
uint16_t PWM_pulsewidth = 10;


void adc_etc0_isr() {
  ADC_ETC_DONE0_1_IRQ |= 1; // clear interrupt source
  unsigned int res = ADC_ETC_TRIG0_RESULT_1_0 & 0xfff;
  Serial.print("from etc0: ");
  Serial.println(res);
  asm("dsb"); // make sure memory accesses complete
}

void adc_etc1_isr() {
  ADC_ETC_DONE0_1_IRQ |= 1 << 16; // clear interrupt source
  unsigned int res = (ADC_ETC_TRIG0_RESULT_1_0 >> 16) & 0xfff;
  Serial.print("from etc1: ");
  Serial.println(res);
  asm("dsb"); // make sure memory accesses complete
}

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
  FLEXPWM2_SM1VAL4 = -64; // adc trigger
  FLEXPWM2_SM1VAL5 = 64; // adc trigger

  FLEXPWM2_OUTEN |= FLEXPWM_OUTEN_PWMA_EN(0x0F); // Activate all A channels
  FLEXPWM2_OUTEN |= FLEXPWM_OUTEN_PWMB_EN(0x0F); // Activate all B channels
  FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_LDOK(0x0F);
        // Give Load Okay for all submodules -> reload setting again
  FLEXPWM2_SM1TCTRL = FLEXPWM_SMTCTRL_OUT_TRIG_EN((1 << 4) | (1 << 5)); 
        // val 4 of Flexpwm sm1 as trigger for PWM_OUT_TRIG0
        // val 5 of Flexpwm sm1 as trigger for PWM_OUT_TRIG1
}

void adc_init() {
  analogRead(0);
  analogRead(1);
  ADC1_CFG |= ADC_CFG_ADTRG; // enable hardware trigger for conversion
  ADC1_HC0 = 0b10000; // set input select to external selection from ADC_ETC
  ADC1_HC1 = 0b10000; // same for hardware trigger control 1
}

void adc_etc_init() {
  ADC_ETC_CTRL &= ~(1 << 31); // trigger software reset of all registers in ADC_ETC
  ADC_ETC_CTRL = (1 << 30) | 1; 
        // bypass touch screen control to adc2 and enable external xbar trigger0
  ADC_ETC_TRIG0_CTRL = ADC_ETC_TRIG_CTRL_TRIG_CHAIN(2);
        // set length of trigger chain 
        // (might be very interesting for many reads back to back)
        
  ADC_ETC_TRIG0_CHAIN_1_0 |= ADC_ETC_TRIG_CHAIN_IE1(0b10);
        // interrupt done1 when segment1 finished
  ADC_ETC_TRIG0_CHAIN_1_0 |= ADC_ETC_TRIG_CHAIN_B2B1;
        // trigger next conversion immediately after segment1 finished
  ADC_ETC_TRIG0_CHAIN_1_0 |= ADC_ETC_TRIG_CHAIN_HWTS1(0b10);
        // hardware trigger select of ADC_TRIG1
  ADC_ETC_TRIG0_CHAIN_1_0 |= ADC_ETC_TRIG_CHAIN_CSEL1(8);
        // select adc channel (A1)
  ADC_ETC_TRIG0_CHAIN_1_0 |= ADC_ETC_TRIG_CHAIN_IE0(0b01);
        // interrupt done0 when segment0 finished
  ADC_ETC_TRIG0_CHAIN_1_0 |= ADC_ETC_TRIG_CHAIN_B2B0;
        // trigger next conversion immediately after segment0 finished
  ADC_ETC_TRIG0_CHAIN_1_0 |= ADC_ETC_TRIG_CHAIN_HWTS0(0b1);
        // hardware trigger select of ADC_TRIG0
  ADC_ETC_TRIG0_CHAIN_1_0 |= ADC_ETC_TRIG_CHAIN_CSEL0(7);
        // select adc channel (A0)
  
  attachInterruptVector(IRQ_ADC_ETC0, adc_etc0_isr);
  NVIC_ENABLE_IRQ(IRQ_ADC_ETC0);
  attachInterruptVector(IRQ_ADC_ETC1, adc_etc1_isr);
  NVIC_ENABLE_IRQ(IRQ_ADC_ETC1);
}

void xbar_connect(unsigned int input, unsigned int output) {
  if (input >= 88) return;
  if (output >= 132) return;
  volatile uint16_t *xbar = &XBARA1_SEL0 + (output / 2);
  uint16_t val = *xbar;
  if (!(output & 1)) {
    val = (val & 0xFF00) | input;
  } else {
    val = (val & 0x00FF) | (input << 8);
  }
  *xbar = val;
}

void xbar_init() {
  CCM_CCGR2 |= CCM_CCGR2_XBAR1(CCM_CCGR_ON);   //turn on clock for xbar1
  xbar_connect(XBARA1_IN_FLEXPWM2_PWM1_OUT_TRIG0, XBARA1_OUT_ADC_ETC_TRIG00); 
        // connect FlexPWM2 submodule1 to adc_etc
}

void setup() {
  flexpwm_init_edgealigned();
  flexpwm_init_signed();
  *(portConfigRegister(4)) = 1; // set pin 4 as output
  *(portConfigRegister(5)) = 1; // set pin 5 as output
  adc_init();
  adc_etc_init();
  xbar_init();
  delay(1000);
  Serial.begin(38400);
  Serial.println("init complete");
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
