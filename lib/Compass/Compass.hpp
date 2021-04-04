#pragma once
#if _EXECUTION_ENVIRONMENT == 0
#include "MainMaze/robot/lib/Serial/Communication/Directions.hxx"
#include "MainMaze/robot/lib/extra/utils/Singleton.hxx"
#include "MainMaze/robot/lib/Driver/Driver.hpp"
#include "MainMaze/robot/lib/Serial/Communication/Walls.hxx"
#else

#include <Driver.hpp>
#include <utils/Singleton.hxx>
#include <Communication/Directions.hxx>
#include <Communication/Walls.hxx>

#endif

class Compass : public Singleton<Compass> {
    Direction direction_ = kTop;

public:
    bool GoTo(Direction objective);

    Walls *GetWalls() const;
};
