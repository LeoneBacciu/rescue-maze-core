#pragma once

#include <chrono>
#include <ctime>


#if _EXECUTION_ENVIRONMENT == 0
#include "MainMaze/robot/lib/Bus/BusConnection.hpp"
#include "MainMaze/robot/lib/extra/utils/Singleton.hxx"
#else

#include <BusConnection.hpp>
#include <utils/Singleton.hxx>
#include <MPU6050_tockn.h>
#include <Madgwick/MadgwickAHRS.h>

#endif

using namespace std::chrono;

class Gyro : public Singleton<Gyro>, BusConnection {
public:
    void Begin(unsigned long refresh);

    float Yaw();

    float Roll();

    float Pitch();

    void Calibrate();

    void Update();

private:
#if _EXECUTION_ENVIRONMENT == 0
    float drift_ = 3;
    bool error_ = true;
    float CalculateError();
    long long last_reset_time_ = 0;
#else
    TwoWire wire;
    MPU6050* imu;
    Madgwick filter;
    unsigned long microsPerReading;
    unsigned long microsPrevious;
#endif
};
