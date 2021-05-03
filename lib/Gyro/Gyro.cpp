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

void Gyro::Update() {
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

void Gyro::Begin(unsigned long refresh) {
    wire.begin((uint8_t) PB11, (uint8_t) PB10);
    wire.setClock(400000);
    imu = new MPU6050(wire);
    imu->begin();
    filter.begin(refresh);
    microsPerReading = 1000000 / refresh;
    microsPrevious = micros();
    Calibrate();
}

float Gyro::Yaw() {
    Update();
    return filter.getYaw();
}

float Gyro::Roll() {
    Update();
    return filter.getRoll();
}

float Gyro::Pitch() {
    Update();
    return filter.getPitch();
}

void Gyro::Calibrate() {
    imu->calcGyroOffsets(false, 100, 100);
}

void Gyro::Update() {
    unsigned long microsNow;
    microsNow = micros();
    if (microsNow - microsPrevious >= microsPerReading) {
        imu->update();
        filter.updateIMU(imu->getGyroX(), imu->getGyroY(), imu->getGyroZ(), imu->getAccX(), imu->getAccY(),
                         imu->getAccZ());
        microsPrevious += microsPerReading;
    }
}

#endif
