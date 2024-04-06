#include "imxrt.h"



enum DoneInterrupt {
  NONE,
  DONE0,
  DONE1,
  DONE2
};

enum HwAvg {
  SAMPLE_4 = 0b00,
  SAMPLE_8 = 0b01,
  SAMPLE_16 = 0b10,
  SAMPLE_32 = 0b11,
};

enum SampleTime {
  PERIOD_3 = 0b000,
  PERIOD_5 = 0b001,
  PERIOD_7 = 0b010,
  PERIOD_9 = 0b011,
  PERIOD_13 = 0b100,
  PERIOD_17 = 0b101,
  PERIOD_21 = 0b110,
  PERIOD_25 = 0b111,
};

enum AdcResolution {
  BIT_8 = 0b00,
  BIT_10 = 0b01,
  BIT_12 = 0b10,
};

enum AdcClockDivider {
  NO_DIV = 0b00,
  DIV_2 = 0b01,
  DIV_4 = 0b10,
  DIV_8 = 0b11
};


struct TrigChainInfo {
  int trig_num;
  int chain_length;
  int* read_pins;
  bool trig_sync;
  int chain_priority;
  bool software_trig;
  DoneInterrupt intr;

  TrigChainInfo() {
    trig_num = 0;
    chain_length = 1;
    read_pins = nullptr;
    trig_sync = false;
    chain_priority = 0;
    software_trig = false;
    intr = NONE;
  }
};


struct AdcEtcBeginInfo {
  int num_chains;
  TrigChainInfo* chains;
  HwAvg adc1_avg;
  HwAvg adc2_avg;
  bool adc1_high_speed;
  bool adc2_high_speed;
  SampleTime adc1_sample_time;
  SampleTime adc2_sample_time;
  AdcClockDivider adc1_clock_div;
  AdcClockDivider adc2_clock_div;
  AdcResolution adc1_resolution;
  AdcResolution adc2_resolution;

  AdcEtcBeginInfo() {
    num_chains = 0;
    chains = nullptr;
    adc1_avg = SAMPLE_4;
    adc2_avg = SAMPLE_4;
    adc1_high_speed = false;
    adc2_high_speed = false;
    adc1_sample_time = PERIOD_7;
    adc2_sample_time = PERIOD_7;
    adc1_clock_div = NO_DIV;
    adc2_clock_div = NO_DIV;
    adc1_resolution = BIT_12;
    adc2_resolution = BIT_12;
  }
};


struct AdcEtc {
  static void begin(AdcEtcBeginInfo &info);
  static uint16_t readSingle(int pin);


};
