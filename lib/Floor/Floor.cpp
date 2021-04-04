#include "Floor.hpp"


#if _EXECUTION_ENVIRONMENT == 0
Floor::FloorType Floor::Read() const
{
	AActor* actor = GetBus()->GetActor();

	FHitResult out_hit;

	FVector start = actor->GetActorLocation() + actor->GetActorRotation().RotateVector(FVector(5, 0, 0));

	FVector end = start + actor->GetActorUpVector() * -1000.0f;

	FCollisionQueryParams collision_params;

	bool is_hit = actor->GetWorld()->LineTraceSingleByChannel(out_hit, start, end, ECC_Visibility,
	                                                          collision_params);
	FloorType result = kWhite;
	if (is_hit)
	{
		ACell* cell = Cast<ACell>(out_hit.GetActor());
		if (cell->IsBlack()) result = kBlack;
		if (cell->IsCheckpoint()) result = kCheckpoint;
	}
	return result;
}
#else

Floor::FloorType Floor::Read() const {
    uint32_t color = analogRead(FLOOR_PIN);
    if (color > 800) return kCheckpoint;
    if (color > 400) return kWhite;
    return kBlack;
}
#endif
