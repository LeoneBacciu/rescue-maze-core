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

void Gyro::Begin() {
    wire.begin();
    wire.setClock(400000);
    mpu.initialize();
    uint8_t devStatus = mpu.dmpInitialize();
    if (devStatus == 0) {
        LoadCalibration();
        mpu.setDMPEnabled(true);
    } else {
        Logger::Error(Source::kGyro, "DMP failed");
    }
}

float Gyro::Yaw() {
    Update();
    return -ypr[0] * RADIANS_TO_DEGREES + 180;
}

float Gyro::Roll() {
    Update();
    return -ypr[2] * RADIANS_TO_DEGREES + 180;
}

float Gyro::Pitch() {
    Update();
    return -ypr[1] * RADIANS_TO_DEGREES + 180;
}

void Gyro::Calibrate(const uint16_t samples) {
    mpu.CalibrateGyro(samples);
    mpu.CalibrateAccel(samples);
    gyroOffsets[0] = mpu.getXGyroOffset();
    gyroOffsets[1] = mpu.getYGyroOffset();
    gyroOffsets[2] = mpu.getZGyroOffset();
    accelOffsets[0] = mpu.getXAccelOffset();
    accelOffsets[1] = mpu.getYAccelOffset();
    accelOffsets[2] = mpu.getZAccelOffset();
    calibrated = true;
}

void Gyro::Update() {
    if (millis() - lastMillis > 100) {
        if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        } else {
            Logger::Error(kGyro, "no data in buffer");
        }
        lastMillis = millis();
    }
}

bool Gyro::IsTilted(const uint8_t threshold) {
    Update();
    const auto v = (abs(-ypr[1] * RADIANS_TO_DEGREES) + abs(-ypr[2] * RADIANS_TO_DEGREES));
//    Logger::Verbose(kGyro, "tilt: %.2f", v);
    return v > threshold;
}

void Gyro::LoadCalibration() {
    if (calibrated) {
        mpu.setXGyroOffset(gyroOffsets[0]);
        mpu.setYGyroOffset(gyroOffsets[1]);
        mpu.setZGyroOffset(gyroOffsets[2]);
        mpu.setXAccelOffset(accelOffsets[0]);
        mpu.setYAccelOffset(accelOffsets[1]);
        mpu.setZAccelOffset(accelOffsets[2]);
    } else {
        Calibrate();
    }
}

int8_t Gyro::IsRamp(const uint8_t pitch_threshold, const uint8_t roll_threshold) {
    const auto pitch = Pitch(), roll = Roll();
    if (abs(pitch - 180) > pitch_threshold) {
//        if (++ramp_buffer > 3) {
        return pitch > 180 ? 1 : -1;
//        }
    }
    return 0;
}

#endif
