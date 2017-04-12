#include "ATSP.h"

ATSP AntTruck(18790, 12830);

void setup() {

  AntTruck.set_motor_pins(6, 5, 9, 10);

  AntTruck.setup(Serial, Serial3, 115200);
}

void loop() {
    AntTruck.run();
}