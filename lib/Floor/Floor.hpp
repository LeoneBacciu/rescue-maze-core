#pragma once

#if _EXECUTION_ENVIRONMENT == 0
#include "MainMaze/robot/lib/Bus/BusConnection.hpp"
#include "MainMaze/robot/lib/extra/utils/Singleton.hxx"
#include "MainMaze/Cell.h"
#else
#include <BusConnection.hpp>
#include <utils/Singleton.hxx>
#include <Logger.hpp>
#define FLOOR_PIN PA7
#endif

class Floor : public Singleton<Floor>, BusConnection
{
public:
	enum FloorType { kBlack, kCheckpoint, kWhite };
	FloorType Read();
    uint32_t ReadRaw() const;

private:
    uint8_t blackCounter = 0;
};
