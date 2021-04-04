#pragma once
#include "MainMaze/robot/lib/Bus/BusConnection.hpp"
#include "MainMaze/robot/lib/extra/utils/Singleton.hxx"
#define delayMicroseconds(t) FPlatformProcess::Sleep((t) / 1000.0);

class Brick : public Singleton<Brick>, BusConnection
{
public:
	void Drop(uint8_t quantity=1);
};
