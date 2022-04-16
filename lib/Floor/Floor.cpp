#include "Floor.hpp"

Floor::FloorType Floor::Read() {
//    return kWhite;
    const uint32_t color = ReadRaw();
    Logger::Verbose(kFloor, "reading floor -> %d, (%d)", color, blackCounter);
    if (color < 40) return kCheckpoint;
    if (color > 250 && color < 320) {
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

#endif
