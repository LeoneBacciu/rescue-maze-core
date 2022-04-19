#include "Compass.hpp"
#include <Notification.hpp>

bool Compass::GoTo(const Direction objective, const bool ignore_current, const bool ignore_next) {
    auto serial = SerialPort::Instance();
    const int difference = direction_ - objective;
    Logger::Info(kCompass, "going from %d to %d, current ignore %d, next ignore %d", direction_, objective,
                 ignore_current, ignore_next);
    if (abs(difference) == 2) {
        Logger::Verbose(kCompass, "180 deg rotation");
        Driver::Rotate(false);
        Driver::Rotate(false);
    } else if (difference != 0) {
        bool side = true;
        if (direction_ == kBottom && objective == kRight) side = false;
        else if (direction_ == kRight && objective == kBottom) side = true;
        else if (difference < 0) side = false;
        Driver::Rotate(side);
    }
    bool ignore_halfway = (difference == 0) || ignore_current;
    if (abs(difference) == 1 && !ignore_current) {
        ignore_halfway = Drop();
    }
    if (ignore_halfway) serial->WriteHalfWayPoint(0);
    else serial->WriteHalfWayPoint(GetSidesCode());
    uint8_t extra_drop = serial->ReadHalfWayDrop();
    if (extra_drop > 0) {
        Logger::Verbose(kCompass, "extra drop %d", extra_drop);
        Drop(extra_drop);
    }
    direction_ = objective;
    const bool success = Driver::Go();
    if (!ignore_next && success) {
        if (Drop()) last_drop = true;
    }
    return success;
}

Walls *Compass::GetWalls() const {
    delay(500);
    auto lasers = Lasers::Instance();
    const uint16_t threshold = cell_dimensions::depth;
    uint16_t tmp_walls[] = {
            lasers->ReadFrontMin(), lasers->ReadL(), lasers->ReadB(), lasers->ReadR()
    }, walls[4];
    for (int i = 0; i < 4; ++i) walls[(i + direction_) % 4] = tmp_walls[i];
    Logger::Info(kCompass, "%d - wall values: %d %d %d %d", direction_, walls[0], walls[1], walls[2], walls[3]);
    return new Walls(walls[0] < threshold, walls[1] < threshold, walls[2] < threshold, walls[3] < threshold);
}

bool Compass::Drop(const uint8_t force) const {
    auto brick = Brick::Instance();
    auto temp = Temp::Instance();
    const auto hot = temp->IsHot();
    bool pulse = false;
    const uint8_t drop_left = ((force >> 4) & 0xf), drop_right = (drop_left > 0) ? 0 : force;
    if (hot.left || drop_left > 1) {
        pulse = true;
        Logger::Info(kCompass, "drop left");
        Driver::Rotate(true);
        brick->Drop((drop_left > 0) ? drop_left - 1 : 1);
        Driver::Rotate(false);
    } else if (hot.right || drop_right > 1) {
        pulse = true;
        Logger::Info(kCompass, "drop right");
        Driver::Rotate(false);
        brick->Drop((drop_right > 0) ? drop_right - 1 : 1);
        Driver::Rotate(true);
    } else {
        Logger::Verbose(kCompass, "no drop");
    }
    if (drop_right > 0 || drop_left > 0) pulse = true;
    if (pulse) Notification::Pulse(5);
    return pulse;
}

uint8_t Compass::GetSidesCode() const {
    auto lasers = Lasers::Instance();
    const auto left = lasers->ReadL() <= cell_dimensions::depth, right = lasers->ReadR() <= cell_dimensions::depth;
    return (left ? 0x10 : 0) | (right ? 0x01 : 0);
}
