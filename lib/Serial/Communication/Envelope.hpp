#pragma once
#include "Directions.hxx"
#include "Walls.hxx"

class InputEnvelope
{
public:
	const Direction direction;
	const bool ignore;
	const bool drop;

	InputEnvelope(const Direction direction, const bool ignore, const bool drop)
		: direction(direction), ignore(ignore), drop(drop)
	{
	}

	static InputEnvelope* FromBytes(uint8_t* data);
};

class OutputEnvelope
{
public:
	const Walls walls;
	const bool black;
	const bool checkpoint;

	OutputEnvelope(const Walls walls, const bool black, const bool checkpoint)
		: walls(walls), black(black), checkpoint(checkpoint)
	{
	}

	void ToBytes(uint8_t* bytes) const;
};
