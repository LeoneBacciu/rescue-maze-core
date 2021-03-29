#include "Robot.hpp"


#if _EXECUTION_ENVIRONMENT == 0
#include "../lib/Compass/Compass.hpp"
#include "../lib/Driver/Driver.hpp"
#include "../lib/Lasers/Lasers.hpp"
#include "../lib/Serial/SerialPort.hpp"
#include "../lib/Temp/Temp.hpp"
#else
#include <Compass.hpp>
#include <Driver.hpp>
#include <Lasers.hpp>
#include <SerialPort.hpp>
#include <Temp.hpp>
#endif

void Robot::Setup()
{
	Lasers* lasers = Lasers::Instance();
	Temp* temps = Temp::Instance();
	SerialPort* serial = SerialPort::Instance();
	Compass* compass = Compass::Instance();
}

void Robot::Main()
{
}
