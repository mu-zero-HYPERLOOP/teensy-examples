#include "linear_encoder.h"



void setup() {
  LinearEncoderBeginInfo beginInfo;
  beginInfo.stride = Distance::from_u32_mm(10);
  beginInfo.left_pin = 3;
  beginInfo.right_pin = 2;
  LinearEncoder::begin(beginInfo);
}



void loop() {

  Velocity v = LinearEncoder::velocity();
  Distance d = LinearEncoder::distance();
  uint32_t count = LinearEncoder::stripe_count();

  Serial.print("[v,d,c] = [");
  Serial.print(v.as_m_per_s());
  Serial.print(",");
  Serial.print(d.as_m());
  Serial.print(",");
  Serial.print(count);
  Serial.println("]");


  delay(100);

}
