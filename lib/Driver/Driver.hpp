﻿#pragma once

#if _EXECUTION_ENVIRONMENT == 0
#define delayMicroseconds(t) FPlatformProcess::Sleep(t / 1000.0);
#define UEDebug
#include "MainMaze/robot/lib/Gyro/Gyro.hpp"
#include "MainMaze/robot/lib/Lasers/Lasers.hpp"
#include "MainMaze/robot/lib/Bus/BusConnection.hpp"
#include "MainMaze/robot/lib/extra/utils/Singleton.hxx"
#include "MainMaze/robot/lib/Serial/Communication/Directions.hxx"
#include "MainMaze/robot/lib/extra/utils/Math.hxx"
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
	const uint8_t max_lateral_compensation_speed_ = 15;
	const uint8_t lateral_compensation_threshold_ = 20;
	const uint8_t lateral_compensation_multiplier_ = 20;
	const uint8_t frontal_compensation_multiplier_ = 2;

	enum Speeds : uint8_t { kSlow=30, kMedium=50, kFast=100 };

public:
	void Rotate(bool right);
	void Go();
    static bool RightTurnCondition(float start, float current, float goal);
    static bool LeftTurnCondition(float start, float current, float goal);

private:
    void SetSpeed(int l, int r);
};
