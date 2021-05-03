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

void Gyro::Begin(unsigned long refresh) {
    wire.begin((uint8_t) PB11, (uint8_t) PB10);
    wire.setClock(400000);

    accCoef = 0.02f;
    gyroCoef = 0.98f;
    writeReg(MPU6050_SMPLRT_DIV, 0x00);
    writeReg(MPU6050_CONFIG, 0x00);
    writeReg(MPU6050_PWR_MGMT_1, 0x01);
    Update(false);
    angleGyroX = 0;
    angleGyroY = 0;
    angleX = angleAccX;
    angleY = angleAccY;
    preInterval = millis();

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
    float gx = 0, gy = 0, gz = 0, ax = 0, ay = 0;
    int16_t rgx, rgy, rgz, rax, ray, raz;
    uint16_t samples = 2000;

    delay(50);
    for (int i = 0; i < samples; i++) {
        wire.beginTransmission(MPU6050_ADDR);
        wire.write(0x3B);
        wire.endTransmission(false);
        wire.requestFrom((int) MPU6050_ADDR, 6);

        rax = (Wire.read() << 8 | Wire.read()) / 16384.f;
        ray = (Wire.read() << 8 | Wire.read()) / 16384.f;
        raz = (Wire.read() << 8 | Wire.read()) / 16384.f;

        ax += atan2(ray, raz + abs(rax)) * 360 / 2.f / PI;
        ay += atan2(rax, raz + abs(ray)) * 360 / -2.f / PI;
    }
    accXoffset = ax / samples;
    accYoffset = ay / samples;

    for (int i = 0; i < samples; i++) {
        wire.beginTransmission(MPU6050_ADDR);
        wire.write(0x43);
        wire.endTransmission(false);
        wire.requestFrom((int) MPU6050_ADDR, 6);

        rgx = wire.read() << 8 | wire.read();
        rgy = wire.read() << 8 | wire.read();
        rgz = wire.read() << 8 | wire.read();

        gx += ((float) rgx) / 131.f;
        gy += ((float) rgy) / 131.f;
        gz += ((float) rgz) / 131.f;
    }
    gyroXoffset = gx / samples;
    gyroYoffset = gy / samples;
    gyroZoffset = gz / samples;
    delay(50);
}

void Gyro::Update(bool filter) {
    unsigned long microsNow;
    microsNow = micros();
    if (microsNow - microsPrevious >= microsPerReading) {
        wire.beginTransmission(MPU6050_ADDR);
        wire.write(0x3B);
        wire.endTransmission(false);
        wire.requestFrom((int) MPU6050_ADDR, 14);

        rawAccX = wire.read() << 8 | wire.read();
        rawAccY = wire.read() << 8 | wire.read();
        rawAccZ = wire.read() << 8 | wire.read();
        rawTemp = wire.read() << 8 | wire.read();
        rawGyroX = wire.read() << 8 | wire.read();
        rawGyroY = wire.read() << 8 | wire.read();
        rawGyroZ = wire.read() << 8 | wire.read();

        temp = (rawTemp + 12412.0) / 340.0;

        accX = ((float) rawAccX) / 16384.0;
        accY = ((float) rawAccY) / 16384.0;
        accZ = ((float) rawAccZ) / 16384.0;

        angleAccX = (atan2(accY, accZ + abs(accX)) * 360 / 2.0 / PI) - accXoffset;
        angleAccY = (atan2(accX, accZ + abs(accY)) * 360 / -2.0 / PI) - accYoffset;

        gyroX = ((float) rawGyroX) / 131.;
        gyroY = ((float) rawGyroY) / 131.;
        gyroZ = ((float) rawGyroZ) / 131.;

        gyroX -= gyroXoffset;
        gyroY -= gyroYoffset;
        gyroZ -= gyroZoffset;

        interval = (millis() - preInterval) * 0.001;

        angleGyroX += gyroX * interval;
        angleGyroY += gyroY * interval;
        angleGyroZ += gyroZ * interval;

        angleX = (gyroCoef * (angleX + gyroX * interval)) + (accCoef * angleAccX);
        angleY = (gyroCoef * (angleY + gyroY * interval)) + (accCoef * angleAccY);
        angleZ = angleGyroZ;

        preInterval = millis();
        if (!filter) return;
        this->filter.updateIMU(gyroX, gyroY, gyroZ, accX, accY, accZ);
        microsPrevious += microsPerReading;
    }
}

void Gyro::writeReg(byte reg, byte data) {
    wire.beginTransmission(MPU6050_ADDR);
    wire.write(reg);
    wire.write(data);
    wire.endTransmission();
}

#endif
