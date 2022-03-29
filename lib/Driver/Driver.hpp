#pragma once

#if _EXECUTION_ENVIRONMENT == 0
#include <chrono>
#define delay(t) FPlatformProcess::Sleep((t) / 1000.0);
#define millis() std::chrono::duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count()
// #define UEDebug
#include "MainMaze/robot/lib/Gyro/Gyro.hpp"
#include "MainMaze/robot/lib/Lasers/Lasers.hpp"
#include "MainMaze/robot/lib/Bus/BusConnection.hpp"
#include "MainMaze/robot/lib/extra/utils/Singleton.hxx"
#include "MainMaze/robot/lib/Serial/Communication/Directions.hxx"
#include "MainMaze/robot/lib/extra/utils/Math.hxx"
#include "MainMaze/robot/lib/Floor/Floor.hpp"
#include "MainMaze/robot/lib/Logger/Logger.hpp"
#else

#include "Arduino.h"

#define PWM_R PA10
#define INV_R2 PB12
#define INV_R1 PB13
#define PWM_L PA9
#define INV_L2 PB14
#define INV_L1 PB15

#include <Gyro.hpp>
#include <Floor.hpp>
#include <Lasers.hpp>
#include <Logger.hpp>
#include <BusConnection.hpp>
#include <utils/Singleton.hxx>
#include <utils/Math.hxx>

#endif
#define C_NEGATE(a, b) (!a&b || a&!b)

class Driver : BusConnection {
    static const uint8_t max_lateral_compensation_speed = 30;
    static const uint8_t lateral_compensation_threshold = 200;
    static const uint8_t lateral_compensation_multiplier = 2;
    static const uint8_t frontal_compensation_multiplier = 2;
    static const int16_t frontLasersBias = -4;

    enum Speeds : uint8_t {
        kBreak = 100, kSlow = 150, kRotateSlow = 120, kMedium = 175, kRotateFast = 125, kFast = 225
    };

public:
    static void Rotate(bool right);

    static bool Go();

    static void SetSpeed(int l, int r);

private:

    static bool AdjustFront(bool right = true);

    static bool RightTurnCondition(float start, float current, float goal);

    static bool RightAdjustCondition(uint16_t l, uint16_t c, uint16_t r);

    static bool LeftTurnCondition(float start, float current, float goal);

    static bool LeftAdjustCondition(uint16_t l, uint16_t c, uint16_t r);

    static bool GoCondition(bool use_front, uint16_t current, uint16_t objective);

    static void CenterCell();

};
