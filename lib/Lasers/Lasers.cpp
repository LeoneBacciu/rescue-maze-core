﻿#include "Lasers.hpp"

#include <utility>


#if _EXECUTION_ENVIRONMENT == 0
void Lasers::Begin()
{
}

void Lasers::StartContinuous()
{
}

void Lasers::StopContinuous()
{
}

uint16_t Lasers::ReadF()
{
	return Read(GetBus()->GetActor()->GetActorForwardVector()) - dimensions::depth / 2;
}

float Lasers::ComputeFrontAngle()
{
	return -atan((ReadFL() - ReadFR()) / dimensions::front_lasers_distance);
}

int16_t Lasers::ComputeFrontDifference()
{
	return ReadFR() - ReadFL();
}

int16_t Lasers::ComputeLateralDifference()
{
	return fmod(ReadL(), cell_dimensions::depth) - fmod(ReadR(), cell_dimensions::depth);
}

uint16_t Lasers::ReadFL()
{
	return Read(GetBus()->GetActor()->GetActorForwardVector(), -dimensions::front_lasers_distance / 2)
		- dimensions::depth / 2;
}

uint16_t Lasers::ReadFR()
{
	return Read(GetBus()->GetActor()->GetActorForwardVector(), dimensions::front_lasers_distance / 2)
		- dimensions::depth / 2;
}

uint16_t Lasers::ReadL()
{
	return Read(GetBus()->GetActor()->GetActorRightVector() * -1)
		- dimensions::width / 2;
}

uint16_t Lasers::ReadR()
{
	return Read(GetBus()->GetActor()->GetActorRightVector())
		- dimensions::width / 2;
}

uint16_t Lasers::ReadB()
{
	return Read(GetBus()->GetActor()->GetActorForwardVector() * -1)
		- dimensions::depth / 2;
}

bool Lasers::IsValidWall(const uint16_t l, const uint16_t c, const uint16_t r, const int tolerance)
{
	const uint16_t mean = std::min(r, l) + abs(r - l) / 2;
	return abs(l - r) < dimensions::front_lasers_distance && mean - tolerance <= c && c <= mean + tolerance;
}

int16_t Lasers::FrontDifference(const uint16_t l, const uint16_t r)
{
	return r - l;
}

uint16_t Lasers::Read(FVector direction, float delta_y, bool draw) const
{
	FHitResult out_hit;

	FVector start = GetBus()->GetActor()->GetActorLocation();
	if (delta_y != 0) start += GetBus()->GetActor()->GetActorRotation().RotateVector(FVector(0, delta_y, 0));

	FVector end = (start + (direction * 800.0f));

	FCollisionQueryParams collision_params;

	bool is_hit = GetBus()->GetActor()->GetWorld()->LineTraceSingleByChannel(out_hit, start, end, ECC_Visibility,
	                                                                         collision_params);

	if (draw)
		DrawDebugLine(GetBus()->GetActor()->GetWorld(), start, FVector(out_hit.Location), FColor::Blue, false, 0.5,
		              0, 0.5);
	float result = 8192;
	if (is_hit)
	{
		result = out_hit.Distance;
	}
	return MakeError(result);
}

float Lasers::MakeError(const float value) const
{
	return value * FMath::RandRange(0.97f, 1.03f);
}

#else

void Lasers::Begin() {
    changeAddress(0);
    laserR.init();
    laserR.setTimeout(40);
    laserR.setMeasurementTimingBudget(40000);
    changeAddress(1);
    laserFR.init();
    laserFR.setTimeout(40);
    laserFR.setMeasurementTimingBudget(40000);
    changeAddress(2);
    laserF.init();
    laserF.setTimeout(40);
    laserF.setMeasurementTimingBudget(40000);
    changeAddress(3);
    laserFL.init();
    laserFL.setTimeout(40);
    laserFL.setMeasurementTimingBudget(40000);

}

void Lasers::StartContinuous() {
    continuous = true;
    changeAddress(0);
    laserR.startContinuous();
    changeAddress(1);
    laserFR.startContinuous();
    changeAddress(2);
    laserF.startContinuous();
    changeAddress(3);
    laserFL.startContinuous();
}

void Lasers::StopContinuous() {
    continuous = false;
    changeAddress(0);
    laserR.stopContinuous();
    changeAddress(1);
    laserFR.stopContinuous();
    changeAddress(2);
    laserF.stopContinuous();
    changeAddress(3);
    laserFL.stopContinuous();
}

float Lasers::ComputeFrontAngle() {
    return -atan((ReadFL() - ReadFR()) / dimensions::front_lasers_distance);
}

int16_t Lasers::ComputeFrontDifference() {
    return ReadFR() - ReadFL();
}

int16_t Lasers::ComputeLateralDifference() {
    return fmod(ReadL(), cell_dimensions::depth) - fmod(ReadR(), cell_dimensions::depth);
}

void Lasers::changeAddress(uint8_t laser) {
    if (laser > 7 || laser == current_bus) return;
    GetBus()->beginTransmission(0x70);
    GetBus()->write(1 << laser);
    GetBus()->endTransmission();
    current_bus = laser;
}


uint16_t Lasers::ReadR() {
    changeAddress(0);
    return continuous ? laserR.readRangeContinuousMillimeters() : laserR.readRangeSingleMillimeters();
}

uint16_t Lasers::ReadFR() {
    changeAddress(1);
    return continuous ? laserFR.readRangeContinuousMillimeters() : laserFR.readRangeSingleMillimeters();
}

uint16_t Lasers::ReadF() {
    changeAddress(2);
    return continuous ? laserF.readRangeContinuousMillimeters() : laserF.readRangeSingleMillimeters();
}

uint16_t Lasers::ReadFL() {
    changeAddress(3);
    return continuous ? laserFL.readRangeContinuousMillimeters() : laserFL.readRangeSingleMillimeters();
}

bool Lasers::IsValidWall(const uint16_t l, const uint16_t c, const uint16_t r, const int tolerance)
{
    const uint16_t mean = std::min(r, l) + abs(r - l) / 2;
    return abs(l - r) < dimensions::front_lasers_distance && mean - tolerance <= c && c <= mean + tolerance;
}

int16_t Lasers::FrontDifference(const uint16_t l, const uint16_t r)
{
    return r - l;
}

uint16_t Lasers::ReadL() {
    return 0;
}

uint16_t Lasers::ReadB() {
    return 0;
}

#endif
