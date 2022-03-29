#include "Brick.hpp"

#if _EXECUTION_ENVIRONMENT == 0
void Brick::Begin() {

}

void Brick::Drop(const uint8_t quantity)
{
    Logger::Info(kBrick, "dropping %d kit", quantity);
    for (int i = 0; i < quantity; ++i)
    {
        GetBus()->Drop();
        delayMicroseconds(1000);
        Logger::Verbose(kBrick, "kit %d dropped", i);
    };
}
#else

void Brick::Begin() {
    servo.attach(SERVO_PIN);
    servo.write(5);
}

void Brick::Drop(uint8_t quantity) {
    Logger::Info(kBrick, "dropping %d kit", quantity);
    delay(500);
    for (uint8_t q = 0; q < quantity; ++q) {
        servo.write(5);
        delay(200);
        const uint8_t delta = 70;
        for (int i = 0; i < delta; ++i) {
            servo.write(i + 5);
            delay(15);
        }
        delay(500);
        for (int i = 0; i < delta; ++i) {
            servo.write(delta + 5 - i);
            delay(5);
        }
    }

}

#endif