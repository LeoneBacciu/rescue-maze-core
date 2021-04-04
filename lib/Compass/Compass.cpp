#include "Compass.hpp"

bool Compass::GoTo(const Direction objective, const bool ignore_current, const bool ignore_next)
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
		if (!ignore_current) Drop();
	}
	direction_ = objective;
	const bool success = Driver::Go();
	if (!ignore_next) Drop();
	return success;
}

Walls* Compass::GetWalls() const
{
	auto lasers = Lasers::Instance();
	const uint8_t threshold = cell_dimensions::depth;
	uint16_t tmp_walls[] = {
		         lasers->ReadF(), lasers->ReadL(), lasers->ReadB(), lasers->ReadR()
	         }, walls[4];
	for (int i = 0; i < 4; ++i) walls[(i + direction_) % 4] = tmp_walls[i];
	return new Walls(walls[0] < threshold, walls[1] < threshold, walls[2] < threshold, walls[3] < threshold);
}

void Compass::Drop(const uint8_t force) const
{
	auto brick = Brick::Instance();
	auto temp = Temp::Instance();
	const auto hot = temp->IsHot();
	if (hot.left || force == 0x10)
	{
		Driver::Rotate(true);
		brick->Drop();
		Driver::Rotate(false);
	}
	else if (hot.right || force == 0x01)
	{
		Driver::Rotate(false);
		brick->Drop();
		Driver::Rotate(true);
	}
}
