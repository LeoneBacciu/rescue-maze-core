#pragma once
#if _EXECUTION_ENVIRONMENT == 0
#include "MainMaze/robot/lib/Serial/Communication/Directions.hxx"
#include "MainMaze/robot/lib/extra/utils/Singleton.hxx"
#include "MainMaze/robot/lib/Driver/Driver.hpp"
#include "MainMaze/robot/lib/Serial/Communication/Walls.hxx"
#include "MainMaze/robot/lib/Brick/Brick.hpp"
#include "MainMaze/robot/lib/Temp/Temp.hpp"
#include "MainMaze/robot/lib/Logger/Logger.hpp"
#else

#include <Driver.hpp>
#include <utils/Singleton.hxx>
#include <Communication/Directions.hxx>
#include <Communication/Walls.hxx>
#include <Temp.hpp>
#include <Brick.hpp>
#include <Logger.hpp>

#endif

class Compass : public Singleton<Compass>
{
	Direction direction_ = kTop;

public:
	bool GoTo(Direction objective, bool ignore_current = true, bool ignore_next = true);

	Walls* GetWalls() const;

	void Drop(uint8_t force = 0) const;
};
