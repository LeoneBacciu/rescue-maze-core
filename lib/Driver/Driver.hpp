#pragma once

#if _EXECUTION_ENVIRONMENT == 0
#define delayMicroseconds(t) FPlatformProcess::Sleep(d / 1000.0);
#include "MainMaze/robot/lib/common/Gyro/Gyro.hpp"
#include "MainMaze/robot/lib/common/Lasers/Lasers.hpp"
#include "MainMaze/robot/lib/common/Bus/BusConnection.hpp"
#include "MainMaze/robot/utils/Singleton.hxx"
#include "MainMaze/robot/data/Directions.hxx"
#include "MainMaze/robot/utils/Math.hxx"
#else
#define PWM_R PB3
#define INV_R1 PB12
#define INV_R2 PB13
#define PWM_L PB4
#define INV_L1 PB14
#define INV_L2 PB15
#include <Gyro.hpp>
#include <Lasers.hpp>
#include <BusConnection.hpp>
#include <utils/Singleton.hxx>
#include <utils/Math.hxx>
#endif

class Driver : public Singleton<Driver>, BusConnection
{
	const uint8_t max_lateral_compensation_speed_ = 6;
	const uint8_t lateral_compensation_threshold_ = 10;
	const uint8_t lateral_compensation_multiplier_ = 20;
	const uint8_t frontal_compensation_multiplier_ = 5;
	const uint8_t stuck_compensation_multiplier_ = 40;
	const uint8_t degree_override_threshold_ = 5;

	enum Speeds : uint8_t { kSlow=30, kMedium=50, kFast=100 };

public:
	void Rotate(bool right);
	void Go();
    static bool RightTurnCondition(float start, float current, float goal);
    static bool LeftTurnCondition(float start, float current, float goal);

private:
    static void SetSpeed(int l, int r);
};
