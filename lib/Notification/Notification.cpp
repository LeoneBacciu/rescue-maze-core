//
// Created by stark on 13/05/21.
//

#include "Notification.hpp"

void Notification::Pulse(uint8_t times) {
    for (int i = 0; i < times; ++i) {
        digitalWrite(PA8, HIGH);
        delay(250);
        digitalWrite(PA8, LOW);
        delay(250);
    }
}
