// ReSharper disable CppUE4CodingStandardNamingViolationWarning
#pragma once

#if _EXECUTION_ENVIRONMENT == 0
#include <tuple>
#else
#include "../../../../../.platformio/packages/toolchain-gccarmnoneeabi/arm-none-eabi/include/c++/9.2.1/tuple"
#endif

enum Direction : uint8_t
{
	kRight = 0,
	kTop = 1,
	kLeft = 2,
	kBottom = 3
};

inline Direction DirectionFromInt(const uint8_t p)
{
	switch (p)
	{
	case 0:
		return kRight;
	case 1:
		return kTop;
	case 2:
		return kLeft;
	default:
		return kBottom;
	}
}

inline Direction DirectionFromBytes(uint8_t* bytes)
{
	if (bytes[0] == 1) return kRight;
	if (bytes[1] == 1) return kTop;
	if (bytes[2] == 1) return kLeft;
	return kBottom;
}

inline char* DirectionToBytes(const Direction direction)
{
	char out[4];
	for (int i = 0; i < 4; ++i)
	{
		out[i] = direction == i ? 1 : 0;
	}
	return out;
}
