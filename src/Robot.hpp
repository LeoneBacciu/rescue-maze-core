#pragma once

#if _EXECUTION_ENVIRONMENT == 0
#include "MainMaze/robot/lib/Compass/Compass.hpp"
#include "MainMaze/robot/lib/Floor/Floor.hpp"
#include "MainMaze/robot/lib/Serial/SerialPort.hpp"
#include "MainMaze/robot/lib/Temp/Temp.hpp"
#else

#include <SerialPort.hpp>
#include <Compass.hpp>
#include <Lasers.hpp>
#include <Gyro.hpp>
#include <Temp.hpp>
#include <Floor.hpp>

#endif


class Robot
{
	static SerialPort* serial_;
	static Compass* compass_;
	static Lasers* lasers_;
	static Gyro* gyro_;
	static Temp* temp_;
	static Floor* floor_;
public:
	static void Setup();

	static bool Main();
};
