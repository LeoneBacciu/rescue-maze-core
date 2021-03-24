#include "Compass.hpp"

void Compass::GoTo(const direction objective)
{
	Driver* driver = Driver::Instance();
	const int difference = direction_ - objective;
	if (abs(difference) == 2)
	{
		driver->Rotate(false);
		driver->Rotate(false);
	}
	else if (difference != 0)
	{
		bool side = true;
		if (direction_ == kBottom && objective == kRight) side = false;
		else if (direction_ == kRight && objective == kBottom) side = true;
		else if (difference < 0) side = false;
		driver->Rotate(side);
	}
	direction_ = objective;
	driver->Go();
}
