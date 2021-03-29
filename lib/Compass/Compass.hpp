#pragma once
#if _EXECUTION_ENVIRONMENT == 0
#include "MainMaze/robot/lib/Serial/Communication/Directions.hxx"
#include "MainMaze/robot/lib/extra/utils/Singleton.hxx"
#include "MainMaze/robot/lib/Driver/Driver.hpp"
#else
#include <Driver.hpp>
#include <utils/Singleton.hxx>
#include <Communication/Directions.hxx>

#endif

class Compass : public Singleton<Compass> {
    Direction direction_ = kTop;

public:
    void GoTo(Direction objective);
};
