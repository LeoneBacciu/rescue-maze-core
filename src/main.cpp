#if _EXECUTION_ENVIRONMENT != 0
#include <Wire.h>
#include <Lasers.hpp>
#include "Arduino.h"
#include "Robot.hpp"

uint32_t start;

void setup() {
    delay(500);
    Wire.begin();
    BusConnection::SetBus(&Wire);

    Robot::Setup();
}

void loop() {
    if (!Robot::Main()) return;
}
#endif
