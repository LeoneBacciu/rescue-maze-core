#include "Temp.hpp"


GeometricPair<bool> Temp::IsHot() {
    const auto measures = Read();
    Logger::Info(kTemp, "%.1f %.1f temp: %.1f, %.1f", threshold_l, threshold_r, measures.left, measures.right);
    return {
            measures.left > threshold_l,
            measures.right > threshold_r
    };
}

#if _EXECUTION_ENVIRONMENT == 0
void Temp::Calibrate() {
    const GeometricPair<float> values = Read();
    threshold = (values.left + values.right) / 2;
    Logger::Info(kTemp, "calibrated threshold: %d", threshold);
}

GeometricPair<float> Temp::Read() {
    return {
        ReadSide(GetBus()->GetActor()->GetActorRightVector() * -1),
        ReadSide(GetBus()->GetActor()->GetActorRightVector())
    };
}


float Temp::ReadSide(FVector direction) const {
    FHitResult out_hit;

    FVector start = GetBus()->GetActor()->GetActorLocation();

    FVector end = start + direction * 1000.0f;

    FCollisionQueryParams collision_params;

    bool is_hit = GetBus()->GetActor()->GetWorld()->LineTraceSingleByChannel(out_hit, start, end, ECC_Visibility,
                                                                             collision_params);
    float result = 0;
    if (is_hit)
    {
        AWall* wall = Cast<AWall>(out_hit.GetActor());
        if (wall) result = (out_hit.Distance < 30) ? wall->temp : 25;
    }
    return result;
}
#else

void Temp::Calibrate() {
    auto data = Read();
    threshold_l = data.left + 2;
    threshold_r = data.right + 2;
}

GeometricPair<float> Temp::Read() {
    return {readTemp(ADDR_L), readTemp(ADDR_R)};
}

float Temp::readTemp(uint8_t addr) {
    GetBus()->beginTransmission(addr); // start transmission to device
    GetBus()->write(0x07);                 // sends register address to ReadF from
    GetBus()->endTransmission(false);   // end transmission
    GetBus()->requestFrom(addr, (uint8_t) 3); // send data n-bytes ReadF
    uint16_t temp = GetBus()->read() | GetBus()->read() << 8;
    __unused uint8_t pec = GetBus()->read();

//    GetBus()->readBytes(buffer, 3);
//    uint16_t temp = uint16_t(buffer[0]) | (uint16_t(buffer[1]) << 8);
    return ((float) temp * .02f) - 273.15;
}

#endif
