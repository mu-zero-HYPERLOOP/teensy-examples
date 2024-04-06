#include <Arduino.h>
#include <cassert>
#include <cstdlib>

#include "can.h"
#include "core_pins.h"
#include "imxrt.h"
#include "wiring.h"

Can1 can1;

void can0_rx(const CAN_message_t &msg) {
  Serial.print("RX:");
  Serial.println(msg.id);
}

void setup() {
  Serial.begin(38400);
  delay(1000);

  CanBeginInfo can1_beginInfo;
  can1_beginInfo.baudrate = CAN_BAUDRATE_1000Kbps;
  can1_beginInfo.loopback = true;
  // Setup filters
  CanFilter onlyOdIds[64];
  for (int i = 0; i < 64; i++) {
    onlyOdIds[i].id = random();
    onlyOdIds[i].mask = random();
    onlyOdIds[i].ide = false;
  }

  can1_beginInfo.filters = onlyOdIds;
  can1_beginInfo.filter_count = 64;

  can1.begin(can1_beginInfo);
}

uint32_t id = 0;
uint32_t rx_id = 0;
uint32_t count = 64;

void loop() {

  CAN_message_t msg;
  msg.id = 1;
  Serial.println("Sending");
  can1.send(msg);

  delay(100);

  can1.recv(msg);
  Serial.println("Received");

  if (msg.id != 1) {
    Serial.println("TEST FAILED");
  } else {
    Serial.println("TEST SUCCESSFUL");
  }
  exit(0);
}
