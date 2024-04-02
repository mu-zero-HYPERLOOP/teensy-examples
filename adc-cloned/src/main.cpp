#include <Arduino.h>
#include "imxrt.h"
#include <ADC.h>

// from SDK PIT XBAR ADC_ETC ADC
// chain A0 A1   AD_B1_02  AD_B1_03   ADC1 IN7 8
#define PRREG(x) Serial.print(#x" 0x"); Serial.println(x,HEX)
uint16_t PWM_SM0_Div = 2; // divider set to 0
uint16_t PWM_range = 500; // PWM range from 0 to 500
uint16_t PWM_pulsewidth = 250; // 50% duty cycle

volatile int New0 = 0, New1 = 0;
volatile uint32_t val0, val1;
// instantiate a new ADC object
ADC *adc = new ADC(); // adc object;

void adcetc0_isr() {
  ADC_ETC_DONE0_1_IRQ |= 1;   // clear
  val0 += ADC_ETC_TRIG0_RESULT_1_0 & 4095;
  New0++;
  Serial.println("hello from isr0");
  asm("dsb");
}

void adcetc1_isr() {
  ADC_ETC_DONE0_1_IRQ |= 1 << 16;   // clear
  val1 += (ADC_ETC_TRIG0_RESULT_1_0 >> 16) & 4095;
  New1++;
  Serial.println("hello from isr1");
  asm("dsb");
}

void adc_init() {
  // init and calibrate with help from core
  analogReadResolution(8);
  analogRead(0);
  analogRead(1);
  ADC1_CFG |= ADC_CFG_ADTRG;   // hardware trigger
  // ADC1_CFG = 0x200b;
  ADC1_HC0 = 16;   // ADC_ETC channel
  ADC1_HC1 = 16;
}

void adc_etc_init() {
  ADC_ETC_CTRL &= ~(1 << 31); // SOFTRST
  ADC_ETC_CTRL = 0x40000001;  // start with trigger 0; TSC_BYPASS: TSC will control ADC2 directly
  ADC_ETC_TRIG0_CTRL = 0x100;   // chainlength -1; TRIG chain length to the ADC Chain = 2
  ADC_ETC_TRIG0_CHAIN_1_0 = 0x50283017;   // ADC1 7 8, chain channel, HWTS, IE, B2B;
  // Chain 1: Finished Interrupt on Done1, Enable B2B, the next ADC trigger will be sent as soon as possible. ADC hardware trigger selection:2, ADC channel selection 8
  // Chain 0: Finished Interrupt on Done0, Enable B2B, the next ADC trigger will be sent as soon as possible. ADC hardware trigger selection:1, ADC channel selection 7
  attachInterruptVector(IRQ_ADC_ETC0, adcetc0_isr);
  NVIC_ENABLE_IRQ(IRQ_ADC_ETC0);
  attachInterruptVector(IRQ_ADC_ETC1, adcetc1_isr);
  NVIC_ENABLE_IRQ(IRQ_ADC_ETC1);
}

void xbar_connect(unsigned int input, unsigned int output)
{
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
  CCM_CCGR2 |= CCM_CCGR2_XBAR1(CCM_CCGR_ON);   //turn clock on for xbara1
  //xbar_connect(56, 103);   // pit to adc_etc
  xbar_connect(XBARA1_IN_FLEXPWM2_PWM1_OUT_TRIG0, XBARA1_OUT_ADC_ETC_TRIG00); //FlexPWM to adc_etc
}

void flewpwm_init() {     //set PWM
  FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_CLDOK(0x0F);//  Clear Load Okay LDOK(SM) -> no reload of PWM settings
  FLEXPWM2_SM0CTRL = FLEXPWM_SMCTRL_FULL | FLEXPWM_SMCTRL_PRSC(PWM_SM0_Div); //Full cycle update (@val1) and prescaler value
  FLEXPWM2_SM0CTRL2 = FLEXPWM_SMCTRL2_INDEP | FLEXPWM_SMCTRL2_CLK_SEL(0); //A & B independant
 
  FLEXPWM2_SM0INIT = 0 ; // start at 0x00
  FLEXPWM2_SM0VAL0 = 0; // mid point for symetrical pulse around 0x00 not used
  FLEXPWM2_SM0VAL1 = PWM_range-1; // end  range
  //PWM A edges
  FLEXPWM2_SM0VAL2 = FLEXPWM2_SM0INIT ; // start of pulse on A, hier bei 0
  FLEXPWM2_SM0VAL3 = FLEXPWM2_SM0INIT + PWM_pulsewidth -1; // end of pulse on A

  FLEXPWM2_SM0VAL4 = 0; // adc trigger

  FLEXPWM2_OUTEN |= FLEXPWM_OUTEN_PWMA_EN(0x0F); // Activate all A channels
  FLEXPWM2_OUTEN |= FLEXPWM_OUTEN_PWMB_EN(0x0F); // Activate all B channels
  FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_LDOK(0x0F);// Load Okay LDOK(SM) -> reload setting again
  FLEXPWM2_SM0TCTRL = FLEXPWM_SMTCTRL_OUT_TRIG_EN(1 << 4); //  val 4 of Flexpwm sm0 as trigger; #define FLEXPWM_SMTCTRL_OUT_TRIG_EN(n)   ((uint16_t)(((n) & 0x3F) << 0))
}

void setup() {
  Serial.begin(250000);
  while (!Serial);
  delay(1000);
  xbar_init();
  adc_init();
  adc_etc_init();
  
  adc->adc0->setAveraging(1); // set number of averages
  adc->adc0->setResolution(8); // set bits of resolution
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED); // change the conversion speed
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED); // change the sampling speed
  flewpwm_init();
  *(portConfigRegister(4)) = 1;   // enable output pin 4
  
  PRREG(ADC1_CFG);
  PRREG(ADC1_HC0);
  PRREG(ADC1_HC1);
  PRREG(ADC_ETC_CTRL);
  PRREG(ADC_ETC_TRIG0_CTRL);
  PRREG(ADC_ETC_TRIG0_CHAIN_1_0);
}

void loop() {
  cli();      //  no interrupts during readout and clear  
  Serial.printf("  %d  %d\n", val0/New0, val1/New1);
  val0=0;     val1=0;   // Clear values for next measurement
  New0 = 0;   New1 = 0;
  sei();
  delay (5);    // allow time for averaging and output
  FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_CLDOK(0x0F);//  Clear Load Okay LDOK(SM) -> no reload of PWM settings
  FLEXPWM2_SM0VAL4++; // shift adc trigger position
  if (FLEXPWM2_SM0VAL4 > PWM_range  - 1) FLEXPWM2_SM0VAL4 = 0;  // end reached  -> start at beginning
  FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_LDOK(0x0F); // Load Okay LDOK(SM) -> reload setting again
}
