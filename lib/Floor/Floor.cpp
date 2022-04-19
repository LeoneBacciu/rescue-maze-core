#include "Floor.hpp"

Floor::FloorType Floor::Read() {
    const uint32_t color = ReadRaw();
    Logger::Verbose(kFloor, "reading floor -> %d, (%d)", color, blackCounter);
    if (color < threshold - 100) return kCheckpoint;
    if (color > threshold + 50 && color < threshold + 150) {
        if (++blackCounter > 2) return kBlack;
        else return kWhite;
    }
    blackCounter = 0;
    return kWhite;
}

#if _EXECUTION_ENVIRONMENT == 0

uint32_t Floor::ReadRaw() const
{
    AActor* actor = GetBus()->GetActor();

    FHitResult out_hit;

    FVector start = actor->GetActorLocation() + actor->GetActorRotation().RotateVector(FVector(5, 0, 0));

    FVector end = start + actor->GetActorUpVector() * -1000.0f;

    FCollisionQueryParams collision_params;

    bool is_hit = actor->GetWorld()->LineTraceSingleByChannel(out_hit, start, end, ECC_Visibility,
                                                              collision_params);
    uint32_t result = 50;
    if (is_hit)
    {
        ACell* cell = Cast<ACell>(out_hit.GetActor());
        if (cell->IsBlack()) result = 850;
        if (cell->IsCheckpoint()) result = 15;
    }
    return result;
}

#else

uint32_t Floor::ReadRaw() const {
    return analogRead(FLOOR_PIN);
}

void Floor::Calibrate() {
    threshold = 0;
    for (int i = 0; i < 10; i++) {
        threshold += ReadRaw();
        delay(10);
    }
    threshold /= 10;
}

#endif
