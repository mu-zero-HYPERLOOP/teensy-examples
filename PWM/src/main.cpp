#include <Arduino.h>

#define PIN_SDC 32
#define PIN_PRECHARGE_DONE 22
#define PIN_PRECHARGE_START 23
#define PIN_VDC_MEAS 20
#define VDC_MEAS_OFFSET 0

#define ANALOG_BITS 12


int counter = 0;

void setup() {
  // put your setup code here, to run once:
  
  analogReadResolution(ANALOG_BITS);
  analogReadAveraging(32);
  

  // initialize pins
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  pinMode(PIN_SDC, OUTPUT);
  digitalWrite(PIN_SDC, LOW);
  
  pinMode(PIN_PRECHARGE_DONE, OUTPUT);
  digitalWrite(PIN_PRECHARGE_DONE, LOW);
  
  pinMode(PIN_PRECHARGE_START, OUTPUT);
  digitalWrite(PIN_PRECHARGE_START, LOW);

  Serial.begin(9600);
}

void loop() {
  // counter increases every 500ms

  // loop 20s
  if(counter > 40) {
    counter = 0;
    Serial.println("Counter reset");
    Serial.println("-----");
  }

  // after 5s activate SDC
  if(counter >= 10) {
    digitalWrite(PIN_SDC, HIGH);
    if(counter == 10) {
      Serial.println("SDC activated");
    }
  } else {
    digitalWrite(PIN_SDC, LOW);
  }

  // after 10s activate Precharge
  if(counter >= 20) {
    digitalWrite(PIN_PRECHARGE_START, HIGH);
    digitalWrite(LED_BUILTIN, HIGH);
    if(counter == 20) {
      Serial.println("Precharge activated");
    }
  } else {
    digitalWrite(PIN_PRECHARGE_START, LOW);
    digitalWrite(LED_BUILTIN, LOW);
  }

  // after 15s activate ext. Relay
  if(counter >= 30) {
    digitalWrite(PIN_PRECHARGE_DONE, HIGH);
    if(counter == 30) {
      Serial.println("Precharge done, activating external Relay");
    }
  } else {
    digitalWrite(PIN_PRECHARGE_DONE, LOW);
  }

  // normalized voltage measurement
  float v_dc_raw_meas = 3.3 * analogRead(PIN_VDC_MEAS) / (1<<ANALOG_BITS) - VDC_MEAS_OFFSET;

  // invert gains of OpAmp, Resistor Divider, AMC1351
  float v_dc = 1.0 * v_dc_raw_meas / 1.5 * (52500/1500) / 0.4; 

  Serial.printf("raw: %f  -  V_DC: %f \n", v_dc_raw_meas, v_dc);


  counter++;
  delay(500);
}

