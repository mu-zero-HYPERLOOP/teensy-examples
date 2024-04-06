#include "adc.hpp"
#include "imxrt.h"


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
  ADC1_HC1 = ADC_HC_ADCH(16); // channel of hardware trigger 1 is controlled by ADC_ETC

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
  ADC2_HC1 = ADC_HC_ADCH(16); // channel of hardware trigger 1 is controlled by ADC_ETC

}




