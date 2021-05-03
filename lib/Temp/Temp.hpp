#pragma once

#if _EXECUTION_ENVIRONMENT == 0
#include "DrawDebugHelpers.h"
#include "MainMaze/Wall.h"
#include "MainMaze/robot/lib/Bus/BusConnection.hpp"
#include "MainMaze/robot/lib/extra/utils/GeometricPair.hxx"
#include "MainMaze/robot/lib/extra/utils/Singleton.hxx"
#include "MainMaze/robot/lib/Logger/Logger.hpp"
#else
#define ADDR_L 0x5A
#define ADDR_R 0x5B

#include "Arduino.h"
#include <BusConnection.hpp>
#include <Logger.hpp>
#include <utils/Singleton.hxx>
#include <utils/GeometricPair.hxx>

#endif


class Temp : public Singleton<Temp>, BusConnection {
public:
    uint16_t threshold = 0;

    void Calibrate();

    GeometricPair<float> Read();

    GeometricPair<bool> IsHot();

private:
#if _EXECUTION_ENVIRONMENT == 0
    float ReadSide(FVector direction) const;
#else

    float readTemp(uint8_t addr);

#endif
};
