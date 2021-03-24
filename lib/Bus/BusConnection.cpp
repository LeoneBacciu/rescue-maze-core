#include "BusConnection.hpp"

#if _EXECUTION_ENVIRONMENT == 0
DrivableActor* BusConnection::bus_ = nullptr;

void BusConnection::SetBus(DrivableActor* actor)
{
    bus_ = actor;
}

DrivableActor* BusConnection::GetBus()
{
    return bus_;
}
#else
TwoWire* BusConnection::bus_ = nullptr;

void BusConnection::SetBus(TwoWire* wire) {
    bus_ = wire;
}

TwoWire* BusConnection::GetBus() {
    return bus_;
}

#endif