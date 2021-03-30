#include "Compass.hpp"

void Compass::GoTo(const Direction objective)
{
	const int difference = direction_ - objective;
	if (abs(difference) == 2)
	{
		Driver::Rotate(false);
		Driver::Rotate(false);
	}
	else if (difference != 0)
	{
		bool side = true;
		if (direction_ == kBottom && objective == kRight) side = false;
		else if (direction_ == kRight && objective == kBottom) side = true;
		else if (difference < 0) side = false;
		Driver::Rotate(side);
	}
	direction_ = objective;
	Driver::Go();
}

void Compass::GetWalls(uint8_t* walls) const
{
	auto lasers = Lasers::Instance();
	uint8_t tmp_walls[] = {lasers->ReadF(), lasers->ReadL(), lasers->ReadB(), lasers->ReadR()};
	for (int i = 0; i < 4; ++i)
	{
		walls[(i + direction_) % 4] = tmp_walls[i];
	}
}
