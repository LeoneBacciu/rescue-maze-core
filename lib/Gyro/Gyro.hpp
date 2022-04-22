#pragma once


#if _EXECUTION_ENVIRONMENT == 0
#include <chrono>
#include <ctime>
#include "MainMaze/robot/lib/Bus/BusConnection.hpp"
#include "MainMaze/robot/lib/extra/utils/Singleton.hxx"
using namespace std::chrono;
#else

#define RADIANS_TO_DEGREES 57.2958

#include <BusConnection.hpp>
#include <utils/Singleton.hxx>
#include <Logger.hpp>
#include "MPU6050_6Axis_MotionApps20.h"

#endif


class Gyro : public Singleton<Gyro>, BusConnection {
public:
    Gyro() : wire(PB11, PB10), mpu(MPU6050_DEFAULT_ADDRESS, &wire) {}

    void Begin();

    float Yaw();

    float Roll();

    float Pitch();

    void Calibrate(uint16_t samples = 10);

    void Update();

    bool IsTilted(uint8_t threshold = 5);

    int8_t IsRamp(uint8_t pitch_threshold = 15, uint8_t roll_threshold = 20);

private:
#if _EXECUTION_ENVIRONMENT == 0
    float drift_ = 1;
    bool error_ = true;
    float CalculateError();
    long long last_reset_time_ = 0;
#else
    TwoWire wire;
    MPU6050 mpu;

    uint8_t fifoBuffer[64]; // FIFO storage buffer

    Quaternion q;
    VectorFloat gravity;
    float ypr[3];

    bool calibrated = false;
    int16_t gyroOffsets[3], accelOffsets[3];

    uint32_t lastMillis = 0;

    int ramp_buffer = 0;

    void LoadCalibration();
#endif
};
