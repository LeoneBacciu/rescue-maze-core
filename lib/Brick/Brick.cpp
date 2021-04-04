#include "Brick.hpp"

#include "MainMaze/robot/lib/Driver/Driver.hpp"

void Brick::Drop(const uint8_t quantity)
{
	for (int i = 0; i < quantity; ++i)
	{
		GetBus()->Drop();
		delayMicroseconds(1000);
	};
}
