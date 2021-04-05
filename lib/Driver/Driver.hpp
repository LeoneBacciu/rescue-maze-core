#pragma once

#if _EXECUTION_ENVIRONMENT == 0
#define delayMicroseconds(t) FPlatformProcess::Sleep((t) / 1000.0);
// #define UEDebug
#include "MainMaze/robot/lib/Gyro/Gyro.hpp"
#include "MainMaze/robot/lib/Lasers/Lasers.hpp"
#include "MainMaze/robot/lib/Bus/BusConnection.hpp"
#include "MainMaze/robot/lib/extra/utils/Singleton.hxx"
#include "MainMaze/robot/lib/Serial/Communication/Directions.hxx"
#include "MainMaze/robot/lib/extra/utils/Math.hxx"
#include "MainMaze/robot/lib/Floor/Floor.hpp"
#else
#define PWM_R PB3
#define INV_R1 PB12
#define INV_R2 PB13
#define PWM_L PB4
#define INV_L1 PB14
#define INV_L2 PB15
#include <Gyro.hpp>
#include <Floor.hpp>
#include <Lasers.hpp>
#include <BusConnection.hpp>
#include <utils/Singleton.hxx>
#include <utils/Math.hxx>
#endif
#define C_NEGATE(a, b) (!a&b || a&!b)

class Driver : BusConnection
{
	static const uint8_t max_lateral_compensation_speed = 15;
	static const uint8_t lateral_compensation_threshold = 20;
	static const uint8_t lateral_compensation_multiplier = 20;
	static const uint8_t frontal_compensation_multiplier = 2;

	enum Speeds : uint8_t { kSlow=30, kMedium=50, kFast=100 };

public:
	static void Rotate(bool right);
	static bool Go();
	static bool RightTurnCondition(float start, float current, float goal);
	static bool LeftTurnCondition(float start, float current, float goal);
	static bool GoCondition(bool use_front, uint16_t current, uint16_t objective);

private:
	static void SetSpeed(int l, int r);
};
