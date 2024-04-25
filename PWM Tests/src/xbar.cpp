#include "xbar.hpp"

#include <imxrt.h>
#include <cinttypes>

void xbar_connect(unsigned int input, unsigned int output)
// from SciMo
// from https://github.com/manitou48/teensy4/blob/master/pitxbaradc.ino
{
  if (input >= 88)
    return;
  if (output >= 132)
    return;
  volatile uint16_t *xbar = &XBARA1_SEL0 + (output / 2);
  uint16_t val = *xbar;
  if (!(output & 1))
  {
    val = (val & 0xFF00) | input;
  }
  else
  {
    val = (val & 0x00FF) | (input << 8);
  }
  *xbar = val;
}


void xbar_init() {CCM_CCGR2 |= CCM_CCGR2_XBAR1(CCM_CCGR_ON);
  CCM_CCGR2 |= CCM_CCGR2_XBAR1(CCM_CCGR_ON);

  // for some reason the PWM submodules in the XBAR inputs are labeled 1-4 instead of 0-3 like everywhere else
  // so FLEXPWM4_PWM1 is submodule 0 of the flexpwm 4 module
  xbar_connect(XBARA1_IN_FLEXPWM4_PWM1_OUT_TRIG0, XBARA1_OUT_FLEXPWM1_PWM3_EXT_SYNC);
  xbar_connect(XBARA1_IN_FLEXPWM4_PWM1_OUT_TRIG0, XBARA1_OUT_FLEXPWM2_PWM0_EXT_SYNC); 
  xbar_connect(XBARA1_IN_FLEXPWM4_PWM1_OUT_TRIG0, XBARA1_OUT_FLEXPWM2_PWM2_EXT_SYNC); 
  xbar_connect(XBARA1_IN_FLEXPWM4_PWM1_OUT_TRIG0, XBARA1_OUT_FLEXPWM2_PWM3_EXT_SYNC); 
  xbar_connect(XBARA1_IN_FLEXPWM4_PWM1_OUT_TRIG0, XBARA1_OUT_FLEXPWM3_EXT_SYNC1);
  xbar_connect(XBARA1_IN_FLEXPWM4_PWM1_OUT_TRIG0, XBARA1_OUT_FLEXPWM4_EXT_SYNC2); 

}
