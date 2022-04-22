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

#define PWM_R PA9
#define INV_R1 PB13
#define INV_R2 PB12
#define PWM_L PA10
#define INV_L1 PB14
#define INV_L2 PB15

#include <BusConnection.hpp>
#include <utils/Singleton.hxx>

#endif
#define C_NEGATE(a, b) (!a&b || a&!b)

class Driver : BusConnection {
    static const uint8_t max_lateral_compensation_speed = 50;
    static const uint8_t lateral_compensation_multiplier = 6;

    static const uint8_t max_lateral_compensation_speed_ramp = 100;
    static const uint8_t lateral_compensation_multiplier_ramp = 10;

    enum Speeds : uint8_t {
//        kBreak = 100, kSlow = 125, kRotateSlow = 150, kMedium = 200, kRotateFast = 250, kFast = 250
        kBreak = 100,
        kBreakHard = 200,
        kSlow = 125,
        kRotateSlow = 150,
        kMedium = 150,
        kRotateFast = 250,
        kFast = 200,
        kVeryFast = 250
    };

public:
    static void Rotate(bool right, bool center_start = true, bool center_end = true);

    static bool Go();

    static void SetSpeed(int l, int r);

    static void Pause();

    static void Resume();

//private:

    static bool AdjustFront(bool right = true);

    static bool RightTurnCondition(float start, float current, float goal);

    static bool RightAdjustCondition(uint16_t l, uint16_t c, uint16_t r);

    static bool LeftTurnCondition(float start, float current, float goal);

    static bool LeftAdjustCondition(uint16_t l, uint16_t c, uint16_t r);

    static bool GoCondition(bool use_front, uint16_t current, uint16_t objective, uint8_t is_ramp, float pitch);

    static bool CenterCell();

    static void Break(int l, int r, int time = 100);

    static void ReturnToAngle(float goal, bool fast = false);

    static void PartialCenter(float start_angle, bool up);

    static bool ExpensiveCenter();

    static int speedL, speedR;

    static bool center_next;
};
