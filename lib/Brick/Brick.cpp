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
}

void Brick::Drop(uint8_t quantity) {
    Logger::Info(kBrick, "dropping %d kit", quantity);
    for (int i = 0; i < quantity; ++i) {
        servo.write(45);
        delay(30);
        servo.write(0);
        delay(30);
        Logger::Verbose(kBrick, "kit %d dropped", i);
    }
}

#endif