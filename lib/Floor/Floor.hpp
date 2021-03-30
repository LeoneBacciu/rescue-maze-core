#pragma once
#if _EXECUTION_ENVIRONMENT == 0
#include "MainMaze/robot/lib/Bus/BusConnection.hpp"
#include "MainMaze/robot/lib/extra/utils/Singleton.hxx"
#include "MainMaze/Cell.h"
#else
#endif

class Floor : public Singleton<Floor>, BusConnection
{
public:
	enum FloorType { kBlack, kCheckpoint, kWhite };
	FloorType Read() const;
};
