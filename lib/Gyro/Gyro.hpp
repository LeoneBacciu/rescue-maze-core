#pragma once

#include <chrono>
#include <ctime>


#if _EXECUTION_ENVIRONMENT == 0
#include "MainMaze/robot/lib/Bus/BusConnection.hpp"
#include "MainMaze/robot/lib/extra/utils/Singleton.hxx"
#else

#define MPU6050_ADDR         0x68
#define MPU6050_SMPLRT_DIV   0x19
#define MPU6050_CONFIG       0x1a
#define MPU6050_PWR_MGMT_1   0x6b

#include <BusConnection.hpp>
#include <utils/Singleton.hxx>
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

    void Update(bool filter=true);

private:
#if _EXECUTION_ENVIRONMENT == 0
    float drift_ = 1;
    bool error_ = true;
    float CalculateError();
    long long last_reset_time_ = 0;
#else
    TwoWire wire;

    int16_t rawAccX, rawAccY, rawAccZ, rawTemp,
            rawGyroX, rawGyroY, rawGyroZ;

    float gyroXoffset, gyroYoffset, gyroZoffset, accXoffset, accYoffset;

    float temp, accX, accY, accZ, gyroX, gyroY, gyroZ;

    float angleGyroX, angleGyroY, angleGyroZ,
            angleAccX, angleAccY, angleAccZ;

    float angleX, angleY, angleZ;

    float interval;
    uint32_t preInterval;

    float accCoef, gyroCoef;

    void writeReg(byte reg, byte data);

    Madgwick filter;
    unsigned long microsPerReading;
    unsigned long microsPrevious;
#endif
};
