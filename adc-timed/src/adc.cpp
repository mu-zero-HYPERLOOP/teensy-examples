#include "adc.hpp"
#include "imxrt.h"
#include "analog.c"
#include "usb_serial.h"


static uint8_t done0_tiggers = 0;
static uint8_t done1_tiggers = 0;
static uint8_t done2_tiggers = 0;

__attribute__((weak)) void adc_etc_done0_isr(AdcTrigRes res) {
  Serial.print(res.trig_res<0, 0>());
  Serial.print("\t");
  Serial.print(res.trig_res<0, 1>());
  Serial.print("\t");
  Serial.print(res.trig_res<0, 2>());
  Serial.print("\t");
  Serial.println(res.trig_res<0, 3>());
}
void adc_etc_0_isr() {
  ADC_ETC_DONE0_1_IRQ |= done0_tiggers;
  AdcTrigRes res;
  adc_etc_done0_isr(res);
  asm("dsb");
}

__attribute__((weak)) void adc_etc_done1_isr(AdcTrigRes res) {

}
void adc_etc_1_isr() {
  ADC_ETC_DONE0_1_IRQ |= done1_tiggers;
  AdcTrigRes res;
  adc_etc_done1_isr(res);
  asm("dsb");
}

__attribute__((weak)) void adc_etc_done2_isr(AdcTrigRes res) {

}
void adc_etc_2_isr() {
  ADC_ETC_DONE2_ERR_IRQ |= done2_tiggers;
  AdcTrigRes res;
  adc_etc_done2_isr(res);
  asm("dsb");
}




void AdcEtc::begin(AdcEtcBeginInfo &info) {
  ADC1_CFG = ADC_CFG_OVWREN; // allows to overwrite unread result from conversion
                             // seems to be necessary for etc trigger to work
  ADC1_CFG |= ADC_CFG_AVGS(info.adc1_avg); // number of hardware averages taken
  ADC1_CFG |= ADC_CFG_ADTRG; // enable hardware trigger for conversion
  if (info.adc1_high_speed) ADC1_CFG |= ADC_CFG_ADHSC; // enable high speed conversion
  ADC1_CFG |= ADC_CFG_ADSTS(info.adc1_sample_time & 0b11); // number of clock cycles for sampling
  ADC1_CFG |= ADC_CFG_ADIV(info.adc1_clock_div); // clock divider
  if (info.adc1_sample_time >> 2) ADC1_CFG |= ADC_CFG_ADLSMP; // enable long sampling time (see ADSTS)
  ADC1_CFG |= ADC_CFG_MODE(info.adc1_resolution); // conversion resolution. 0b10 = 12bit
  ADC1_CFG |= ADC_CFG_ADICLK(0b00); // input clock select
  ADC2_HC0 = ADC_HC_ADCH(16); // channel of hardware trigger 1 is controlled by ADC_ETC
  ADC1_HC1 = ADC_HC_ADCH(16); // channel of hardware trigger is controlled by ADC_ETC
  ADC1_HC2 = ADC_HC_ADCH(16); // channel of hardware trigger is controlled by ADC_ETC
  ADC1_HC3 = ADC_HC_ADCH(16); // channel of hardware trigger is controlled by ADC_ETC
  ADC1_HC4 = ADC_HC_ADCH(16); // channel of hardware trigger is controlled by ADC_ETC
  ADC1_HC5 = ADC_HC_ADCH(16); // channel of hardware trigger is controlled by ADC_ETC
  ADC1_HC6 = ADC_HC_ADCH(16); // channel of hardware trigger is controlled by ADC_ETC
  ADC1_HC7 = ADC_HC_ADCH(16); // channel of hardware trigger is controlled by ADC_ETC

  ADC2_CFG = ADC_CFG_OVWREN; // allows to overwrite unread result from conversion
                             // seems to be necessary for etc trigger to work
  ADC2_CFG |= ADC_CFG_AVGS(info.adc2_avg); // number of hardware averages taken
  ADC2_CFG |= ADC_CFG_ADTRG; // enable hardware trigger for conversion
  if (info.adc2_high_speed) ADC2_CFG |= ADC_CFG_ADHSC; // enable high speed conversion
  ADC2_CFG |= ADC_CFG_ADSTS(info.adc2_sample_time & 0b11); // number of clock cycles for sampling
  ADC2_CFG |= ADC_CFG_ADIV(info.adc2_clock_div); // clock divider
  if (info.adc2_sample_time >> 2) ADC2_CFG |= ADC_CFG_ADLSMP; // enable long sampling time (see ADSTS)
  ADC2_CFG |= ADC_CFG_MODE(info.adc2_resolution); // conversion resolution. 0b10 = 12bit
  ADC2_CFG |= ADC_CFG_ADICLK(0b00); // input clock select
  ADC2_HC0 = ADC_HC_ADCH(16); // channel of hardware trigger 1 is controlled by ADC_ETC
  ADC2_HC1 = ADC_HC_ADCH(16); // channel of hardware trigger 1 is controlled by ADC_ETC
  ADC2_HC2 = ADC_HC_ADCH(16); // channel of hardware trigger is controlled by ADC_ETC
  ADC2_HC3 = ADC_HC_ADCH(16); // channel of hardware trigger is controlled by ADC_ETC
  ADC2_HC4 = ADC_HC_ADCH(16); // channel of hardware trigger is controlled by ADC_ETC
  ADC2_HC5 = ADC_HC_ADCH(16); // channel of hardware trigger is controlled by ADC_ETC
  ADC2_HC6 = ADC_HC_ADCH(16); // channel of hardware trigger is controlled by ADC_ETC
  ADC2_HC7 = ADC_HC_ADCH(16); // channel of hardware trigger is controlled by ADC_ETC

  ADC_ETC_CTRL = ADC_ETC_CTRL_SOFTRST;
  ADC_ETC_CTRL &= ~ADC_ETC_CTRL_SOFTRST;
        // trigger soft reset of all registers in ADC_ETC
  ADC_ETC_CTRL |= ADC_ETC_CTRL_TSC_BYPASS;
  for (int chain = 0; chain < info.num_chains; chain++) {
    ADC_ETC_CTRL |= ADC_ETC_CTRL_TRIG_ENABLE(1 << info.chains[chain].trig_num);

    // ETC_TRIG control register:
    if (info.chains[chain].trig_sync) IMXRT_ADC_ETC.TRIG[chain].CTRL = ADC_ETC_TRIG_CTRL_SYNC_MODE;
    IMXRT_ADC_ETC.TRIG[chain].CTRL |= ADC_ETC_TRIG_CTRL_TRIG_PRIORITY(info.chains[chain].chain_priority & 0b111);
    IMXRT_ADC_ETC.TRIG[chain].CTRL |= ADC_ETC_TRIG_CTRL_TRIG_CHAIN((info.chains[chain].chain_length - 1) & 0b111);
    if (info.chains[chain].software_trig) IMXRT_ADC_ETC.TRIG[chain].CTRL |= ADC_ETC_TRIG_CTRL_SW_TRIG;

    // ETC_TRIG chain registers:
    for (int segm = 0; segm < info.chains[chain].chain_length; segm++) {
      *((&(IMXRT_ADC_ETC.TRIG[chain].CHAIN_1_0)) + ((segm / 2) * 4)) = 0;
            // reset trigger chain config
      if (segm % 2) {
        *((&(IMXRT_ADC_ETC.TRIG[chain].CHAIN_1_0)) + ((segm / 2) * 4)) 
          |= ADC_ETC_TRIG_CHAIN_IE1(segm == info.chains[chain].chain_length - 1 
              ? info.chains[chain].intr : DoneInterrupt::NONE);
            // enable done interrupt only for last segment
        *((&(IMXRT_ADC_ETC.TRIG[chain].CHAIN_1_0)) + ((segm / 2) * 4)) 
          |= ADC_ETC_TRIG_CHAIN_CSEL1(pin_to_channel[info.chains[chain].read_pins[segm]]);
            // select read channel
      } else {
        *((&(IMXRT_ADC_ETC.TRIG[chain].CHAIN_1_0)) + ((segm / 2) * 4)) 
          |= ADC_ETC_TRIG_CHAIN_IE0(segm == info.chains[chain].chain_length - 1 
              ? info.chains[chain].intr : DoneInterrupt::NONE);
            // enable done interrupt only for last segment
        *((&(IMXRT_ADC_ETC.TRIG[chain].CHAIN_1_0)) + ((segm / 2) * 4)) 
          |= ADC_ETC_TRIG_CHAIN_CSEL0(pin_to_channel[info.chains[chain].read_pins[segm]]);
            // select read channel
      }
      *((&(IMXRT_ADC_ETC.TRIG[chain].CHAIN_1_0)) + ((segm / 2) * 4)) 
        |= ADC_ETC_TRIG_CHAIN_B2B1 | ADC_ETC_TRIG_CHAIN_B2B0;
            // trigger next segments conversion immediatedly after segment finished
      *((&(IMXRT_ADC_ETC.TRIG[chain].CHAIN_1_0)) + ((segm / 2) * 4)) 
        |= ADC_ETC_TRIG_CHAIN_HWTS1(1 << info.chains[chain].trig_num) 
        | ADC_ETC_TRIG_CHAIN_HWTS0(1 << info.chains[chain].trig_num);
            // select harware trigger for ADC
    }
    switch (info.chains[chain].intr) {
      case DoneInterrupt::DONE0:
        attachInterruptVector(IRQ_ADC_ETC0, adc_etc_0_isr);
        break;
      case DoneInterrupt::DONE1:
        attachInterruptVector(IRQ_ADC_ETC0, adc_etc_1_isr);
        break;
      case DoneInterrupt::DONE2:
        attachInterruptVector(IRQ_ADC_ETC0, adc_etc_2_isr);
        break;
      default:
        break;
    }
  }
}




