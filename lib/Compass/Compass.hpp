#pragma once
#if _EXECUTION_ENVIRONMENT == 0
#include "MainMaze/robot/data/Directions.hxx"
#include "MainMaze/robot/utils/Singleton.hxx"
#include "MainMaze/robot/lib/common/Driver/Driver.hpp"
#else
#include <Driver.hpp>
#include <utils/Singleton.hxx>
#include <Communication/Directions.hxx>

#endif

class Compass : public Singleton<Compass> {
    direction direction_ = kTop;

public:
    void GoTo(direction objective);
};
