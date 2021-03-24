#pragma once

#if _EXECUTION_ENVIRONMENT == 0
#include "DrawDebugHelpers.h"
#include "MainMaze/Wall.h"
#include "MainMaze/robot/lib/common/Bus/BusConnection.hpp"
#include "MainMaze/robot/utils/GeometricPair.hxx"
#include "MainMaze/robot/utils/Singleton.hxx"
#else
#define ADDR_L 0x5A
#define ADDR_R 0x5B
#include "Arduino.h"
#include <BusConnection.hpp>
#include <utils/Singleton.hxx>
#include <utils/GeometricPair.hxx>
#endif


class Temp : public Singleton<Temp>, BusConnection
{
public:
	uint16_t threshold = 0;
	void Calibrate();
	GeometricPair<uint16_t> Read();
	GeometricPair<bool> IsHot();

#if _EXECUTION_ENVIRONMENT == 0
private:
	float ReadSide(FVector direction) const;
#else
    uint16_t readTemp(uint8_t addr);
    uint16_t read16(uint8_t addr, uint8_t reg);
#endif
};
