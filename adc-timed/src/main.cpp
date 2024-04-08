#include "core_pins.h"
#include "imxrt.h"
#include "pins_arduino.h"
#include "wiring.h"
#include <Arduino.h>
#include "adc.hpp"

#define PRREG(x) Serial.print(#x" 0x"); Serial.println(x,HEX)

uint16_t PWM_SM0_Div = 4; // refer to page 3147 of reference manual
uint16_t PWM_range = 500; // PWM range from 0 to 500
uint16_t PWM_pulsewidth = 10;
volatile uint16_t val0, val1, val2, val3;


void adc_etc0_isr() {
  ADC_ETC_DONE0_1_IRQ |= ADC_ETC_DONE0_1_IRQ_TRIG_DONE0(0); // clear interrupt source TRIG0_DONE0
  val0 = ADC_ETC_TRIG0_RESULT_1_0 & 0xfff;
  val1 = (ADC_ETC_TRIG0_RESULT_1_0 >> 16) & 0xfff;
  val2 = ADC_ETC_TRIG0_RESULT_3_2 & 0xfff;
  val3 = (ADC_ETC_TRIG0_RESULT_3_2 >> 16) & 0xfff;
  Serial.print(val0);
  Serial.print("\t");
  Serial.print(val1);
  Serial.print("\t");
  Serial.print(val2);
  Serial.print("\t");
  Serial.println(val3);
  asm("dsb"); // make sure memory accesses complete
}

void adc_etc1_isr() {
  ADC_ETC_DONE0_1_IRQ |= ADC_ETC_DONE0_1_IRQ_TRIG_DONE1(0); // clear interrupt source TRIG0_DONE1
  //val1 = (ADC_ETC_TRIG0_RESULT_1_0 >> 16) & 0xfff;
  asm("dsb"); // make sure memory accesses complete
}

void flexpwm_init_edgealigned() {     //set PWM
  FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_CLDOK(0xf);
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

  FLEXPWM2_OUTEN |= FLEXPWM_OUTEN_PWMA_EN(0b0001); // Activate all A channels
  FLEXPWM2_OUTEN |= FLEXPWM_OUTEN_PWMB_EN(0b0001); // Activate all B channels
  FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_LDOK(0x0F);
        // Give Load Okay for all submodules -> reload setting again
  FLEXPWM2_SM0TCTRL = FLEXPWM_SMTCTRL_OUT_TRIG_EN((1 << 4)); 
        // val 4 of Flexpwm sm0 as trigger for PWM_OUT_TRIG0
        // val 5 of Flexpwm sm0 as trigger for PWM_OUT_TRIG1
}

void flexpwm_init_signed() {     //set PWM
  FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_CLDOK(0x0F);
        // Clear Load Okay for all submodules -> no reload of PWM settings
  FLEXPWM2_SM1CTRL = FLEXPWM_SMCTRL_FULL | FLEXPWM_SMCTRL_PRSC(0b111); 
        // Full cycle update (@val1) and prescaler value
  FLEXPWM2_SM1CTRL2 = FLEXPWM_SMCTRL2_INDEP | FLEXPWM_SMCTRL2_CLK_SEL(0b00); 
        // A & B independant and use IPBus clock for counter/prescaler
  FLEXPWM2_SM1INIT = 0x8000; // value to start at after reset
                             // should be max negative value
  PRREG(FLEXPWM2_SM1INIT);
                         
  FLEXPWM2_SM1VAL0 = 0; // mid point for symetrical pulse around 0x00 not used
  FLEXPWM2_SM1VAL1 = 0x7fff; // value at which counter resets
                             // should be max positive value
  // PWM A edges
  FLEXPWM2_SM1VAL2 = -200; // start of pulse on A
  FLEXPWM2_SM1VAL3 = 200; // end of pulse on A
  // PWM B edges
  FLEXPWM2_SM1VAL4 = -202; // adc trigger
  FLEXPWM2_SM1VAL5 = 64; // adc trigger

  FLEXPWM2_OUTEN |= FLEXPWM_OUTEN_PWMA_EN(0b0010); // Activate all A channels
  FLEXPWM2_OUTEN |= FLEXPWM_OUTEN_PWMB_EN(0b0010); // Activate all B channels
  FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_LDOK(0x0F);
        // Give Load Okay for all submodules -> reload setting again
  FLEXPWM2_SM1TCTRL = FLEXPWM_SMTCTRL_OUT_TRIG_EN((1 << 4)); 
        // val 4 of Flexpwm2 sm1 as trigger for PWM_OUT_TRIG0
        // val 5 of Flexpwm2 sm1 as trigger for PWM_OUT_TRIG1
}

void adc_init() {
  analogRead(0);
  analogRead(1);
  ADC1_CFG = ADC_CFG_OVWREN; // allows to overwrite unread result from conversion
                             // seems to be necessary for etc trigger to work
  ADC1_CFG |= ADC_CFG_AVGS(0b00); // number of hardware averages taken
  ADC1_CFG |= ADC_CFG_ADTRG; // enable hardware trigger for conversion
  ADC1_CFG |= ADC_CFG_ADHSC; // enable high speed conversion
  ADC1_CFG |= ADC_CFG_ADSTS(0b01); // number of clock cycles for sampling
  ADC1_CFG |= ADC_CFG_ADIV(0b00); // clock divider
  // ADC1_CFG |= ADC_CFG_ADLSMP; // enable long sampling time (see ADSTS)
  ADC1_CFG |= ADC_CFG_MODE(0b10); // conversion resolution. 0b10 = 12bit
  ADC1_CFG |= ADC_CFG_ADICLK(0b00); // input clock select
  ADC1_HC0 = ADC_HC_ADCH(16); // set input select to external selection from ADC_ETC
  ADC1_HC1 = ADC_HC_ADCH(16); // same for hardware trigger control 1
}


void adc_etc_init() {
  ADC_ETC_CTRL = ADC_ETC_CTRL_SOFTRST; 
  ADC_ETC_CTRL &= ~ADC_ETC_CTRL_SOFTRST;
        // trigger software reset of all registers in ADC_ETC
  ADC_ETC_CTRL |= ADC_ETC_CTRL_TSC_BYPASS; // bypass touch screen control to adc2
  ADC_ETC_CTRL |= ADC_ETC_CTRL_TRIG_ENABLE(0b00000001);
        // select external xbar triggers

  ADC_ETC_TRIG0_CTRL = ADC_ETC_TRIG_CTRL_TRIG_CHAIN(0b011);
        // set length of trigger chain 
        
  ADC_ETC_TRIG0_CHAIN_1_0 = ADC_ETC_TRIG_CHAIN_IE1(0b00);
        // select interrupt after segment finished
  ADC_ETC_TRIG0_CHAIN_1_0 |= ADC_ETC_TRIG_CHAIN_B2B1;
        // trigger next conversion immediately after segment finished
  ADC_ETC_TRIG0_CHAIN_1_0 |= ADC_ETC_TRIG_CHAIN_HWTS1(0b00000010);
        // hardware trigger select
  ADC_ETC_TRIG0_CHAIN_1_0 |= ADC_ETC_TRIG_CHAIN_CSEL1(8);
        // select adc channel -> pin 15
  ADC_ETC_TRIG0_CHAIN_1_0 |= ADC_ETC_TRIG_CHAIN_IE0(0b00);
        // select interrupt after segment finished
  ADC_ETC_TRIG0_CHAIN_1_0 |= ADC_ETC_TRIG_CHAIN_B2B0;
        // trigger next conversion immediately after segment finished
  ADC_ETC_TRIG0_CHAIN_1_0 |= ADC_ETC_TRIG_CHAIN_HWTS0(0b00000010);
        // hardware trigger select
  ADC_ETC_TRIG0_CHAIN_1_0 |= ADC_ETC_TRIG_CHAIN_CSEL0(7);
        // select adc channel -> pin 14
  
  ADC_ETC_TRIG0_CHAIN_3_2 = ADC_ETC_TRIG_CHAIN_IE1(0b01);
        // interrupt done0 when segment3 finished
  ADC_ETC_TRIG0_CHAIN_3_2 |= ADC_ETC_TRIG_CHAIN_B2B1;
  ADC_ETC_TRIG0_CHAIN_3_2 |= ADC_ETC_TRIG_CHAIN_HWTS1(0b00000010);
  ADC_ETC_TRIG0_CHAIN_3_2 |= ADC_ETC_TRIG_CHAIN_CSEL1(11);
        // select adc channel -> pin 17
  ADC_ETC_TRIG0_CHAIN_3_2 |= ADC_ETC_TRIG_CHAIN_IE0(0b00);
  ADC_ETC_TRIG0_CHAIN_3_2 |= ADC_ETC_TRIG_CHAIN_B2B0;
  ADC_ETC_TRIG0_CHAIN_3_2 |= ADC_ETC_TRIG_CHAIN_HWTS0(0b00000010);
  ADC_ETC_TRIG0_CHAIN_3_2 |= ADC_ETC_TRIG_CHAIN_CSEL0(12);
        // select adc channel -> pin 16
  attachInterruptVector(IRQ_ADC_ETC0, adc_etc0_isr);
  NVIC_ENABLE_IRQ(IRQ_ADC_ETC0);
  //attachInterruptVector(IRQ_ADC_ETC1, adc_etc1_isr);
  //NVIC_ENABLE_IRQ(IRQ_ADC_ETC1);
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
  xbar_connect(XBARA1_IN_FLEXPWM2_PWM2_OUT_TRIG0, XBARA1_OUT_ADC_ETC_TRIG00);
  //xbar_connect(XBARA1_IN_FLEXPWM2_PWM2_OUT_TRIG1, XBARA1_OUT_ADC_ETC_TRIG00);
  //xbar_connect(XBARA1_IN_FLEXPWM2_PWM1_OUT_TRIG0, XBARA1_OUT_ADC_ETC_TRIG00);
}

void setup() {
  Serial.begin(38400);
  delay(5000);
  xbar_init();

  int pins[4] = {14,15,16,17};
  TrigChainInfo chain1 = TrigChainInfo();
  chain1.trig_num = 0;
  chain1.chain_length = 4;
  chain1.read_pins = pins;
  chain1.trig_sync = false;
  chain1.chain_priority = 3;
  chain1.software_trig = false;
  chain1.intr = DONE0;
  AdcEtcBeginInfo begin = AdcEtcBeginInfo();
  begin.num_chains = 1;
  begin.chains = &chain1;
  //AdcEtc::begin(begin);

  adc_init();
  adc_etc_init();
  flexpwm_init_signed();
  //flexpwm_init_edgealigned();
  *(portConfigRegister(4)) = 1; // set pin 4 as output
  *(portConfigRegister(5)) = 1; // set pin 5 as output

  Serial.println("init complete");
  PRREG(ADC1_CFG);
  PRREG(ADC1_HC0);
  PRREG(ADC1_HC1);
  PRREG(ADC_ETC_CTRL);
  PRREG(ADC_ETC_TRIG0_CTRL);
  PRREG(ADC_ETC_TRIG0_CHAIN_1_0);
  PRREG(ADC_ETC_TRIG0_CHAIN_3_2);
  PRREG(FLEXPWM2_SM0CTRL);
  PRREG(FLEXPWM2_SM0CTRL2);
  PRREG(FLEXPWM2_OUTEN);
}


void loop() {
  //FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_CLDOK(0x0f);

  //FLEXPWM2_SM0VAL3++;
  //if (FLEXPWM2_SM0VAL3 > FLEXPWM2_SM0VAL1) FLEXPWM2_SM0VAL3 = FLEXPWM2_SM0INIT;

  //FLEXPWM2_SM1VAL2++;
  //if ((int16_t)FLEXPWM2_SM1VAL2 >= 0) FLEXPWM2_SM1VAL2 = FLEXPWM2_SM1INIT;
  //FLEXPWM2_SM1VAL3 = -(int16_t)FLEXPWM2_SM1VAL2;

  //FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_LDOK(0x0f);

  //Serial.print("sm0 val2: ");
  //Serial.println(FLEXPWM2_SM0VAL2);
  //Serial.print("sm0 val3: ");
  //Serial.println(FLEXPWM2_SM0VAL3);

  //Serial.print("sm1 val2: ");
  //Serial.println((int16_t) FLEXPWM2_SM1VAL2);
  //Serial.print("sm1 val3: ");
  //Serial.println(FLEXPWM2_SM1VAL3);
  delay(10);
  //Serial.print("val0 ");
  //Serial.println(val0);
  //Serial.print("val1 ");
  //Serial.println(val1);
  //Serial.println("loop still alive");



  //ADC_ETC_TRIG0_CTRL |= ADC_ETC_TRIG_CTRL_TRIG_MODE;
  //ADC_ETC_TRIG0_CTRL |= ADC_ETC_TRIG_CTRL_SW_TRIG;

  //Serial.println("aa");


}
