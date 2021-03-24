#pragma once


#if _EXECUTION_ENVIRONMENT == 0
#include "MainMaze/DrivableActor.h"
#else
#include <Wire.h>
#endif

class BusConnection
{
#if _EXECUTION_ENVIRONMENT == 0
	static DrivableActor* bus_;
public:
	static void SetBus(DrivableActor* actor);
	static DrivableActor* GetBus();
#else
	static TwoWire* bus_;
public:
    static void SetBus(TwoWire* wire);
    static TwoWire* GetBus();
#endif
};
