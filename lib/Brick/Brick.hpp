#pragma once

#if _EXECUTION_ENVIRONMENT == 0
#include "MainMaze/robot/lib/Driver/Driver.hpp"
#include "MainMaze/robot/lib/Bus/BusConnection.hpp"
#include "MainMaze/robot/lib/extra/utils/Singleton.hxx"
#define delayMicroseconds(t) FPlatformProcess::Sleep((t) / 1000.0);
#else
#include <utils/Singleton.hxx>
#include <BusConnection.hpp>
#include <Driver.hpp>
#include <Servo.h>
#define SERVO_PIN 12
#endif

class Brick : public Singleton<Brick>, BusConnection
{
public:
    void Begin();
	void Drop(uint8_t quantity=1);
#if _EXECUTION_ENVIRONMENT != 0
private:
	Servo servo;
#endif
};
