#include "Brick.hpp"

#include "MainMaze/robot/lib/Logger/Logger.hpp"


#if _EXECUTION_ENVIRONMENT == 0
void Brick::Begin() {

}

void Brick::Drop(const uint8_t quantity)
{
    for (int i = 0; i < quantity; ++i)
    {
        Logger::Info(kBrick, "dropping kit...");
        GetBus()->Drop();
        Logger::Info(kBrick, "kit dropped");
        delayMicroseconds(1000);
    };
}
#else

void Brick::Begin() {
    servo.attach(SERVO_PIN);
}

void Brick::Drop(uint8_t quantity) {
    for (int i = 0; i < quantity; ++i) {
        servo.write(45);
        delay(30);
        servo.write(0);
        delay(30);
    }
}

#endif