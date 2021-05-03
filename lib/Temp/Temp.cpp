#include "Temp.hpp"


GeometricPair<bool> Temp::IsHot() {
    const auto measures = Read();
    Logger::Info(kTemp, "temp: %.3f, %.3f", measures.left, measures.right);
    return {
        measures.left > threshold,
        measures.right > threshold
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
    threshold = (data.left + data.right) / 2;
}

GeometricPair<float> Temp::Read() {
    return {readTemp(ADDR_L), readTemp(ADDR_R)};
}

float Temp::readTemp(uint8_t addr) {
    uint16_t ret;
    GetBus()->beginTransmission(addr); // start transmission to device
    GetBus()->write(7);                 // sends register address to ReadF from
    GetBus()->endTransmission(false);   // end transmission
    GetBus()->requestFrom(addr, (uint8_t) 3); // send data n-bytes ReadF
    ret = GetBus()->read() | GetBus()->read() << 8;
    __unused uint8_t pec = GetBus()->read();
    return ((float) ret - 546.3f) * .2f;
}

#endif
