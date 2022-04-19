#pragma once


#if _EXECUTION_ENVIRONMENT == 0
#include "MainMaze/robot/lib/Bus/BusConnection.hpp"
#include "MainMaze/robot/lib/extra/utils/Singleton.hxx"
#include "MainMaze/robot/lib/extra/utils/Constants.hxx"
#include "DrawDebugHelpers.h"
#else

#include <utils/Singleton.hxx>
#include <utils/Constants.hxx>
#include <BusConnection.hpp>
#include <VL53L0X/VL53L0X.h>
#include <Logger.hpp>

#endif

class Lasers : public Singleton<Lasers>, BusConnection {
public:
    void Begin();

    void StartContinuous();

    void StopContinuous();

    float ComputeFrontAngle();

    uint16_t ReadF();

    int16_t ComputeFrontDifference();

    int16_t ComputeLateralDifference(uint16_t threshold = 1000, int16_t bias = 0);

    int16_t ComputeVerticalDifference(uint16_t threshold = 1000, int16_t bias = 0);

    uint16_t ReadFL();

    uint16_t ReadFR();

    uint16_t ReadL();

    uint16_t ReadR();

    uint16_t ReadB();

    uint16_t ReadFront();

    uint16_t ReadFrontMin();

    static bool IsValidWall(uint16_t l, uint16_t c, uint16_t r, uint16_t tolerance = 10);

    static int16_t FrontDifference(uint16_t l, uint16_t r, int16_t bias = 4);

private:
#if _EXECUTION_ENVIRONMENT == 0
    uint16_t Read(FVector direction, float delta_y = 0, bool draw=false) const;
    float MakeError(float value) const;
#else
    bool continuous = false;
    uint8_t current_bus = -1;
    VL53L0X laserR;
    bool rHighPrecision = true;
    VL53L0X laserFR;
    bool frHighPrecision = true;
    VL53L0X laserF;
    bool fHighPrecision = true;
    VL53L0X laserFL;
    bool flHighPrecision = true;
    VL53L0X laserL;
    bool lHighPrecision = true;
    VL53L0X laserB;
    bool bHighPrecision = true;

    uint8_t frontCounter = 0;

    void changeAddress(uint8_t laser);

    uint16_t Read(VL53L0X laser, uint8_t address, bool *highPrecision);

    struct ADDRESSES {
        static const uint8_t R = 5;
        static const uint8_t FR = 7;
        static const uint8_t F = 6;
        static const uint8_t FL = 4;
        static const uint8_t L = 1;
        static const uint8_t B = 0;
    };


    struct BIASES {
        static const int8_t R = -5;
        static const int8_t FR = -5;
        static const int8_t F = -16;
        static const int8_t FL = 0;
        static const int8_t L = -15;
        static const int8_t B = -16;
    };

#endif
};
