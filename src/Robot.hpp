#pragma once
#include "MainMaze/robot/lib/Compass/Compass.hpp"
#include "MainMaze/robot/lib/Serial/SerialPort.hpp"


class Robot
{
	static SerialPort* serial_;
	static Compass* compass_;
public:
	static void Setup();
	static bool Main();
};
