#include <climits>
#include "Gyro.hpp"

#if _EXECUTION_ENVIRONMENT == 0
void Gyro::Begin(unsigned long refresh) {
    const auto now = high_resolution_clock::now();
    last_reset_time_ = duration_cast<milliseconds>(now.time_since_epoch()).count();
}

float Gyro::Yaw() {
    return FRotator::ClampAxis(360 - GetBus()->GetActor()->GetActorRotation().Yaw + CalculateError());
}

float Gyro::Roll() {
    return FRotator::ClampAxis(360 - GetBus()->GetActor()->GetActorRotation().Roll + CalculateError());
}

float Gyro::Pitch() {
    return FRotator::ClampAxis(360 - GetBus()->GetActor()->GetActorRotation().Pitch + CalculateError());
}

void Gyro::Calibrate() {
    last_reset_time_ = std::chrono::duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

void Gyro::Update(bool filter) {
}

float Gyro::CalculateError() {
    if (error_)
    {
        const auto millis = std::chrono::duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
        const auto current_error = drift_ * (millis - last_reset_time_) / 600000;
        return current_error;
    }
    return 0.0;
}
#else

void Gyro::Begin(unsigned long refresh, bool calibrate) {
    wire.begin();
    mpu.initialize();
    uint8_t devStatus = mpu.dmpInitialize();
    if (devStatus == 0) {
        mpu.CalibrateGyro(10);
        mpu.CalibrateAccel(10);
        mpu.setDMPEnabled(true);
    } else {
        Logger::Error(Source::kGyro, "DMP failed");
    }
}

float Gyro::Yaw() {
    Update();
    return ypr[0] * RADIANS_TO_DEGREES;
}

float Gyro::Roll() {
    Update();
    return ypr[2] * RADIANS_TO_DEGREES;
}

float Gyro::Pitch() {
    Update();
    return ypr[1] * RADIANS_TO_DEGREES;
}

void Gyro::Calibrate(const uint16_t samples) {
//    mpu.resetDMP();
}

void Gyro::Update() {
    mpu.dmpGetCurrentFIFOPacket(fifoBuffer);
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
}

#endif
