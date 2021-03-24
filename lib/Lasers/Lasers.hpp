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
#endif

class Lasers : public Singleton<Lasers>, BusConnection
{
public:
    void Begin();
    void StartContinuous();
    void StopContinuous();
	float ComputeFrontAngle();
    uint16_t ReadF();
	int16_t ComputeFrontDifference();
	int16_t ComputeLateralDifference();
	uint16_t ReadFL();
	uint16_t ReadFR();
	uint16_t ReadL();
	uint16_t ReadR();
	uint16_t ReadB();

	static bool IsValidWall(uint16_t l, uint16_t c, uint16_t r, int tolerance=5);
	static int16_t FrontDifference(uint16_t l, uint16_t r);

private:
#if _EXECUTION_ENVIRONMENT == 0
	uint16_t Read(FVector direction, float delta_y = 0, bool draw=false) const;
	float MakeError(float value) const;
#else
    bool continuous = false;
    int current_bus = -1;
    VL53L0X laserR;
    VL53L0X laserFR;
    VL53L0X laserF;
    VL53L0X laserFL;
    void changeAddress(uint8_t laser);
#endif
};
