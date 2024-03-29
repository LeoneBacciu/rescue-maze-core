#pragma once

#include <cstdint>

class Walls
{
public:
	const bool front;
	const bool right;
	const bool left;
	const bool back;

	Walls(const bool right, const bool front, const bool left, const bool back)
		: front(front), right(right), left(left), back(back)
	{
	}

	void ToBytes(uint8_t* bytes) const
	{
		bytes[0] = right;
		bytes[1] = front;
		bytes[2] = left;
		bytes[3] = back;
	}
};
